/*
 * libtpms TPM driver
 *
 * Copyright (C) 2013 IBM Corporation
 *
 * Authors:
 *  Stefan Berger   <stefanb@us.ibm.com>
 *  Corey Bryant    <coreyb@linux.vnet.ibm.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "sysemu/tpm_backend.h"
#include "tpm_int.h"
#include "tpm_nvram.h"
#include "qapi/qmp/qerror.h"
#include "migration/migration.h"
#include "sysemu/tpm_backend_int.h"

#include <libtpms/tpm_library.h>
#include <libtpms/tpm_error.h>
#include <libtpms/tpm_memory.h>
#include <libtpms/tpm_nvfilename.h>
#include <libtpms/tpm_tis.h>

/* #define DEBUG_TPM */

#ifdef DEBUG_TPM
#define DPRINTF(fmt, ...) \
    do { fprintf(stderr, fmt, ## __VA_ARGS__); } while (0)
#define DPRINTF_BUFFER(buffer, len) \
    do { tpm_ltpms_dump_buffer(stderr, buffer, len); } while (0)
#else
#define DPRINTF(fmt, ...) \
    do { } while (0)
#define DPRINTF_BUFFER(buffer, len) \
    do { } while (0)
#endif

#define NVRAM_BLOB_OFFSET_FROM_ENTRY(entry_offset) \
    (entry_offset + sizeof(uint32_t))

#define TYPE_TPM_LIBTPMS "tpm-libtpms"
#define TPM_LIBTPMS(obj) \
    OBJECT_CHECK(TPMLTPMsState, (obj), TYPE_TPM_LIBTPMS)

static const TPMDriverOps tpm_ltpms_driver;

/* data structures */
typedef struct TPMLTPMsThreadParams {
    TPMState *tpm_state;

    TPMRecvDataCB *recv_data_callback;
    TPMBackend *tb;
} TPMLTPMsThreadParams;

struct NVRAMEntry {
    uint32_t cur_size;
    uint8_t *buffer;
};

typedef struct NVRAMEntry NVRAMEntry;

struct TPMLTPMsState {
    TPMBackend parent;

    TPMBackendThread tbt;

    TPMLTPMsThreadParams tpm_thread_params;

    bool tpm_initialized;
    bool had_fatal_error;

    BlockDriverState *bdrv;

    NVRAMEntry *perm_state_entry;
    NVRAMEntry *save_state_entry;
    NVRAMEntry *vola_state_entry;

    uint32_t perm_state_entry_offset;
    uint32_t save_state_entry_offset;
    uint32_t vola_state_entry_offset;

    uint32_t perm_state_max_size;
    uint32_t save_state_max_size;
    uint32_t vola_state_max_size;

    QemuMutex tpm_initialized_mutex;

    uint8_t locty; /* locality of command being executed by libtpms */
};

typedef struct TPMLTPMsState TPMLTPMsState;

static TPMBackend *tpm_backend;

/* functions */

#ifdef DEBUG_TPM
static inline void tpm_ltpms_dump_buffer(FILE *stream, unsigned char *buffer,
                                         unsigned int len)
{
    int i;

    for (i = 0; i < len; i++) {
        if (i && !(i % 16)) {
            fprintf(stream, "\n");
        }
        fprintf(stream, "%.2X ", buffer[i]);
    }
    fprintf(stream, "\n");
}
#endif

static inline void tpm_ltpms_free_nvram_entry(NVRAMEntry **entry)
{
    if (*entry) {
        TPM_Free((unsigned char *)*entry);
        *entry = NULL;
    }
}

static inline void tpm_ltpms_free_nvram_buffer(NVRAMEntry *entry)
{
    if (entry && entry->buffer) {
        TPM_Free(entry->buffer);
        entry->buffer = NULL;
        entry->cur_size = 0;
    }
}

static inline void tpm_ltpms_free_nvram_all(TPMLTPMsState *tpm_ltpms)
{
    tpm_ltpms_free_nvram_buffer(tpm_ltpms->perm_state_entry);
    tpm_ltpms_free_nvram_buffer(tpm_ltpms->save_state_entry);
    tpm_ltpms_free_nvram_buffer(tpm_ltpms->vola_state_entry);

    tpm_ltpms_free_nvram_entry(&tpm_ltpms->perm_state_entry);
    tpm_ltpms_free_nvram_entry(&tpm_ltpms->save_state_entry);
    tpm_ltpms_free_nvram_entry(&tpm_ltpms->vola_state_entry);
}

/*
 * Calls into libtpms to get a runtime property of the TPM
 */
static int tpmlib_get_prop(enum TPMLIB_TPMProperty prop)
{
    int result;

    TPM_RESULT res = TPMLIB_GetTPMProperty(prop, &result);
    assert(res == TPM_SUCCESS);

    return result;
}

/*
 * Generates the drive offsets where NVRAM blobs are stored.  Each offset
 * allows for enough room to store the current blob size plus a blob of
 * the maximum size.
 */
static void tpm_ltpms_get_nvram_offsets(TPMLTPMsState *tpm_ltpms)
{
    tpm_ltpms->perm_state_entry_offset = 0;
    tpm_ltpms->perm_state_max_size = tpmlib_get_prop(TPMPROP_TPM_MAX_NV_SPACE);

    tpm_ltpms->save_state_entry_offset =
        ROUND_UP(tpm_ltpms->perm_state_entry_offset + sizeof(uint32_t) +
                 tpm_ltpms->perm_state_max_size + 1, 1024);
    tpm_ltpms->save_state_max_size =
        tpmlib_get_prop(TPMPROP_TPM_MAX_SAVESTATE_SPACE);

    tpm_ltpms->vola_state_entry_offset =
        ROUND_UP(tpm_ltpms->save_state_entry_offset + sizeof(uint32_t) +
                 tpm_ltpms->save_state_max_size + 1, 1024);
    tpm_ltpms->vola_state_max_size =
        tpmlib_get_prop(TPMPROP_TPM_MAX_VOLATILESTATE_SPACE);
}

/*
 * Writes an NVRAM entry and it's blob to the specified drive offset
 */
static int tpm_ltpms_write_to_nvram(TPMLTPMsState *tpm_ltpms, uint32_t offset,
                                    NVRAMEntry *entry, uint32_t max_size)
{
    int rc;
    uint8_t *buffer = entry->buffer;
    uint32_t size = entry->cur_size;
    BlockDriverState *bdrv = tpm_ltpms->bdrv;

    DPRINTF("tpm_libtpms: Writing NVRAM entry to offset %"PRIu32"\n", offset);

    if (tpm_ltpms->had_fatal_error) {
        return TPM_FAIL;
    }

    if (size > max_size) {
        qerror_report(ERROR_CLASS_GENERIC_ERROR, "TPM NVRAM blob size too big");
        return TPM_FAIL;
    }

    DPRINTF("tpm_libtpms: current blob size = %"PRIu32"\n", size);

    /* Write the blob */
    if (size > 0) {
        DPRINTF_BUFFER(buffer, size);

        rc = tpm_nvram_bdrv_write(bdrv, NVRAM_BLOB_OFFSET_FROM_ENTRY(offset),
                                  buffer, size);
        if (rc != size) {
            qerror_report(ERROR_CLASS_GENERIC_ERROR, "TPM NVRAM write failed");
            return rc;
        }
    }

    /* Blob size is stored on disk in big-endian */
    size = cpu_to_be32(size);

    /* Write the blob size */
    rc = tpm_nvram_bdrv_write(bdrv, offset, (uint8_t *)&size, sizeof(size));
    if (rc != sizeof(size)) {
        qerror_report(ERROR_CLASS_GENERIC_ERROR, "TPM NVRAM write failed");
        return rc;
    }

    return TPM_SUCCESS;
}

/*
 * Reads an NVRAM entry and it's blob from the specified drive offset
 */
static int tpm_ltpms_read_from_nvram(TPMLTPMsState *tpm_ltpms, uint32_t offset,
                                     NVRAMEntry **entry, uint32_t max_size)
{
    int rc;
    uint8_t *buffer = NULL;
    uint32_t *size = NULL;
    BlockDriverState *bdrv = tpm_ltpms->bdrv;

    DPRINTF("tpm_libtpms: Reading NVRAM entry from offset %"PRIu32"\n", offset);

    if (tpm_ltpms->had_fatal_error) {
        return TPM_FAIL;
    }

    /* Allocate the in-memory blob entry */
    rc = TPM_Malloc((unsigned char **)entry, sizeof(**entry));
    if (rc != TPM_SUCCESS) {
        qerror_report(ERROR_CLASS_GENERIC_ERROR,
                      "TPM memory allocation failed");
        abort();
    }

    /* Read the blob size */
    rc = tpm_nvram_bdrv_read(bdrv, offset, (uint8_t **)&size, sizeof(*size));
    if (rc != sizeof(*size)) {
        qerror_report(ERROR_CLASS_GENERIC_ERROR, "TPM NVRAM read failed");
        goto err_exit;
    }

    /* Blob size is stored on disk in big-endian */
    *size = be32_to_cpu(*size);

    if (*size > max_size) {
        qerror_report(ERROR_CLASS_GENERIC_ERROR, "TPM NVRAM blob size too big");
        rc = TPM_FAIL;
        goto err_exit;
    }

    DPRINTF("tpm_libtpms: current blob size = %"PRIu32"\n", *size);

    (*entry)->cur_size = *size;
    (*entry)->buffer = NULL;

    /* Read the blob */
    if (*size > 0) {
        rc = tpm_nvram_bdrv_read(bdrv, NVRAM_BLOB_OFFSET_FROM_ENTRY(offset),
                                 &buffer, *size);
        if (rc != *size) {
            qerror_report(ERROR_CLASS_GENERIC_ERROR, "TPM NVRAM read failed");
            goto err_exit;
        }

        (*entry)->buffer = buffer;

        DPRINTF_BUFFER(buffer, *size);
    }

    rc = TPM_SUCCESS;

err_exit:
    if (size) {
        TPM_Free((uint8_t *)size);
    }

    return rc;
}

/*
 * Loads the TPM's NVRAM state from NVRAM drive into memory
 */
static int tpm_ltpms_load_tpm_state_from_nvram(TPMLTPMsState *tpm_ltpms)
{
    int rc;

    rc = tpm_ltpms_read_from_nvram(tpm_ltpms,
                                   tpm_ltpms->perm_state_entry_offset,
                                   &tpm_ltpms->perm_state_entry,
                                   tpm_ltpms->perm_state_max_size);
    if (rc) {
        goto err_exit;
    }

    rc = tpm_ltpms_read_from_nvram(tpm_ltpms,
                                   tpm_ltpms->save_state_entry_offset,
                                   &tpm_ltpms->save_state_entry,
                                   tpm_ltpms->save_state_max_size);
    if (rc) {
        goto err_exit;
    }

    rc = tpm_ltpms_read_from_nvram(tpm_ltpms,
                                   tpm_ltpms->vola_state_entry_offset,
                                   &tpm_ltpms->vola_state_entry,
                                   tpm_ltpms->vola_state_max_size);
    if (rc) {
        goto err_exit;
    }

    return 0;

err_exit:
    tpm_ltpms->had_fatal_error = true;

    return rc;
}

/*
 * Processes a command request by calling into libtpms, and returns
 * result to front end
 */
static void tpm_ltpms_process_request(TPMLTPMsState *tpm_ltpms,
                                      TPMLTPMsThreadParams *thr_parms)
{
    TPM_RESULT res;
    uint32_t in_len, out_len;
    uint8_t *in, *out;
    uint32_t resp_size;
    TPMLocality *locty_data;

    DPRINTF("tpm_libtpms: processing command\n");

    tpm_ltpms->locty = thr_parms->tpm_state->locty_number;

    locty_data = thr_parms->tpm_state->locty_data;

    in      = locty_data->w_buffer.buffer;
    in_len  = locty_data->w_offset;
    out     = locty_data->r_buffer.buffer;
    out_len = locty_data->r_buffer.size;

    if (tpm_ltpms->tpm_initialized) {
        DPRINTF("tpm_libtpms: received %d bytes from VM in locality %d\n",
                in_len, tpm_ltpms->locty);
        DPRINTF_BUFFER(in, in_len);

        resp_size = 0;

        res = TPMLIB_Process(&out, &resp_size, &out_len, in, in_len);
        if (res == TPM_SUCCESS) {
            goto send_response;
        }
        qerror_report(ERROR_CLASS_GENERIC_ERROR,
                      "TPM libtpms command processing failed");
    } else {
        qerror_report(ERROR_CLASS_GENERIC_ERROR,
                      "TPM libtpms not initialized");
    }

    resp_size = tpm_write_fatal_error_response(out, out_len);

send_response:
    DPRINTF("tpm_libtpms: sending %d bytes to TPM front-end\n", resp_size);
    DPRINTF_BUFFER(out, resp_size);

    thr_parms->recv_data_callback(thr_parms->tpm_state, tpm_ltpms->locty, true);

    return;
}

static void tpm_ltpms_worker_thread(gpointer data, gpointer user_data)
{
    TPM_RESULT res;
    TPMLTPMsThreadParams *thr_parms = user_data;
    TPMLTPMsState *tpm_ltpms = TPM_LIBTPMS(thr_parms->tb);
    TPMBackendCmd cmd = (TPMBackendCmd)data;

    tpm_backend = thr_parms->tb;

    DPRINTF("tpm_libtpms: processing command type %d\n", cmd);

    switch (cmd) {
    case TPM_BACKEND_CMD_TPM_RESET:
        if (tpm_ltpms->tpm_initialized) {
            qemu_mutex_lock(&tpm_ltpms->tpm_initialized_mutex);
            tpm_ltpms->tpm_initialized = false;
            qemu_mutex_unlock(&tpm_ltpms->tpm_initialized_mutex);

            TPMLIB_Terminate();
        }
        /* fall through */
    case TPM_BACKEND_CMD_INIT:
        //added by 6u6a
        if(TPMLIB_ChooseTPMVersion(TPMLIB_TPM_VERSION_2) == TPM_SUCCESS){
            fprintf(stdout, "choose tpm2.0\n");
        }
        //end of add
        res = TPMLIB_MainInit();
        if (res == TPM_SUCCESS) {
            qemu_mutex_lock(&tpm_ltpms->tpm_initialized_mutex);
            tpm_ltpms->tpm_initialized = true;
            qemu_mutex_unlock(&tpm_ltpms->tpm_initialized_mutex);
        } else {
            qerror_report(ERROR_CLASS_GENERIC_ERROR,
                          "TPM libtpms initialization failed");
            abort();
        }
        break;
    case TPM_BACKEND_CMD_PROCESS_CMD:
        tpm_ltpms_process_request(tpm_ltpms, thr_parms);
        break;
    case TPM_BACKEND_CMD_END:
        if (tpm_ltpms->tpm_initialized) {
            qemu_mutex_lock(&tpm_ltpms->tpm_initialized_mutex);
            tpm_ltpms->tpm_initialized = false;
            qemu_mutex_unlock(&tpm_ltpms->tpm_initialized_mutex);

            TPMLIB_Terminate();
            tpm_ltpms_free_nvram_all(tpm_ltpms);
        }
        break;
    }
}

/*****************************************************************
 * libtpms TPM library callbacks
 ****************************************************************/

/*
 * Called by libtpms before any access to persistent storage is done
 */
static TPM_RESULT tpm_ltpms_nvram_init(void)
{
    int rc;
    TPMLTPMsState *tpm_ltpms = TPM_LIBTPMS(tpm_backend);
    tpm_ltpms->bdrv = bdrv_lookup_bs(tpm_backend->nvram_id, tpm_backend->nvram_id, NULL);
    if (!tpm_ltpms->bdrv) {
        qerror_report(ERROR_CLASS_GENERIC_ERROR, "TPM 'nvram' drive not found");
        abort();
    }

    rc = tpm_nvram_bdrv_init(tpm_ltpms->bdrv);
    if (rc) {
        qerror_report(ERROR_CLASS_GENERIC_ERROR, "TPM NVRAM drive init failed");
        abort();
    }

    tpm_ltpms_get_nvram_offsets(tpm_ltpms);
    
    rc = tpm_ltpms_load_tpm_state_from_nvram(tpm_ltpms);
    if (rc) {
        qerror_report(ERROR_CLASS_GENERIC_ERROR, "TPM NVRAM load state failed");
        abort();
    }

    return TPM_SUCCESS;
}

/*
 * Called by libtpms when the TPM wants to load state from persistent
 * storage
 */
static TPM_RESULT tpm_ltpms_nvram_loaddata(unsigned char **data,
                                           uint32_t *length,
                                           uint32_t tpm_number,
                                           const char *name)
{
    TPM_RESULT rc = TPM_SUCCESS;
    TPMLTPMsState *tpm_ltpms = TPM_LIBTPMS(tpm_backend);
    NVRAMEntry **entry = NULL;

    DPRINTF("tpm_libtpms: Loading NVRAM state '%s' from storage\n", name);

    if (tpm_ltpms->had_fatal_error) {
        return TPM_FAIL;
    }

    *length = 0;

    if (!strcmp(name, TPM_PERMANENT_ALL_NAME)) {
        entry = &tpm_ltpms->perm_state_entry;
    } else if (!strcmp(name, TPM_SAVESTATE_NAME)) {
        entry = &tpm_ltpms->save_state_entry;
    } else if (!strcmp(name, TPM_VOLATILESTATE_NAME)) {
        entry = &tpm_ltpms->vola_state_entry;
    }

    /* In-memory entries are allocated for the life of the backend */
    assert(entry != NULL);

    *length = (*entry)->cur_size;
    if (*length > 0) {
        rc = TPM_Malloc(data, *length);
        if (rc == TPM_SUCCESS) {
            memcpy(*data, (*entry)->buffer, *length);
        } else {
            qerror_report(ERROR_CLASS_GENERIC_ERROR,
                          "TPM memory allocation failed");
            abort();
        }
    }

    if (*length == 0) {
        rc = TPM_RETRY;
    }

    DPRINTF("tpm_libtpms: Read %"PRIu32" bytes from storage\n", *length);

    return rc;
}

/*
 * Called by libtpms when the TPM wants to store state to persistent
 * storage
 */
static TPM_RESULT tpm_ltpms_nvram_storedata(const unsigned char *data,
                                            uint32_t length,
                                            uint32_t tpm_number,
                                            const char *name)
{
    TPM_RESULT rc = TPM_SUCCESS;
    TPMLTPMsState *tpm_ltpms = TPM_LIBTPMS(tpm_backend);
    NVRAMEntry *entry = NULL;
    uint32_t offset = 0, max_size = 0;

    DPRINTF("tpm_libtpms: Storing NVRAM state '%s' to storage\n", name);

    if (tpm_ltpms->had_fatal_error) {
        return TPM_FAIL;
    }

    if (!strcmp(name, TPM_PERMANENT_ALL_NAME)) {
        entry = tpm_ltpms->perm_state_entry;
        offset = tpm_ltpms->perm_state_entry_offset;
        max_size = tpm_ltpms->perm_state_max_size;
    } else if (!strcmp(name, TPM_SAVESTATE_NAME)) {
        entry = tpm_ltpms->save_state_entry;
        offset = tpm_ltpms->save_state_entry_offset;
        max_size = tpm_ltpms->save_state_max_size;
    } else if (!strcmp(name, TPM_VOLATILESTATE_NAME)) {
        entry = tpm_ltpms->vola_state_entry;
        offset = tpm_ltpms->vola_state_entry_offset;
        max_size = tpm_ltpms->vola_state_max_size;
    }

    /* In-memory entries are allocated for the life of the backend */
    assert(entry != NULL);

    if (length > 0) {
        rc = TPM_Realloc(&entry->buffer, length);
        if (rc != TPM_SUCCESS) {
            qerror_report(ERROR_CLASS_GENERIC_ERROR,
                          "TPM memory allocation failed");
            abort();
        }
        memcpy(entry->buffer, data, length);
        entry->cur_size = length;
    } else {
        tpm_ltpms_free_nvram_buffer(entry);
    }

    if (tpm_ltpms_write_to_nvram(tpm_ltpms, offset, entry, max_size)) {
        goto err_exit;
    }

    DPRINTF("tpm_libtpms: Wrote %"PRIu32" bytes to storage\n", length);

    return rc;

err_exit:
    tpm_ltpms->had_fatal_error = true;

    return TPM_FAIL;
}

/*
 * Called by libtpms when the TPM wants to delete state from persistent
 * storage
 */
static TPM_RESULT tpm_ltpms_nvram_deletename(uint32_t tpm_number,
                                             const char *name,
                                             TPM_BOOL mustExist)
{
    TPMLTPMsState *tpm_ltpms = TPM_LIBTPMS(tpm_backend);
    NVRAMEntry *entry = NULL;
    uint32_t offset = 0, max_size = 0;

    DPRINTF("tpm_libtpms: Deleting NVRAM state '%s' from storage\n", name);

    if (tpm_ltpms->had_fatal_error) {
        return TPM_FAIL;
    }

    if (!strcmp(name, TPM_PERMANENT_ALL_NAME)) {
        entry = tpm_ltpms->perm_state_entry;
        offset = tpm_ltpms->perm_state_entry_offset;
        max_size = tpm_ltpms->perm_state_max_size;
    } else if (!strcmp(name, TPM_SAVESTATE_NAME)) {
        entry = tpm_ltpms->save_state_entry;
        offset = tpm_ltpms->save_state_entry_offset;
        max_size = tpm_ltpms->save_state_max_size;
    } else if (!strcmp(name, TPM_VOLATILESTATE_NAME)) {
        entry = tpm_ltpms->vola_state_entry;
        offset = tpm_ltpms->vola_state_entry_offset;
        max_size = tpm_ltpms->vola_state_max_size;
    }

    if (entry) {
        tpm_ltpms_free_nvram_buffer(entry);

        if (tpm_ltpms_write_to_nvram(tpm_ltpms, offset, entry, max_size)) {
            goto err_exit;
        }
    }

    DPRINTF("tpm_libtpms: Deleted NVRAM state '%s' from storage\n", name);

    return TPM_SUCCESS;

err_exit:
    tpm_ltpms->had_fatal_error = true;

    return TPM_FAIL;
}

/*
 * Called by libtpms to initialize the I/O subsystem of the TPM
 */
static TPM_RESULT tpm_ltpms_io_init(void)
{
    return TPM_SUCCESS;
}

/*
 * Called by libtpms when the TPM needs to determine the locality under
 * which a command is supposed to be executed
 */
static TPM_RESULT tpm_ltpms_io_getlocality(TPM_MODIFIER_INDICATOR *
                                           localityModifier,
                                           uint32_t tpm_number)
{
    TPMLTPMsState *tpm_ltpms = TPM_LIBTPMS(tpm_backend);

    *localityModifier = (TPM_MODIFIER_INDICATOR)tpm_ltpms->locty;

    return TPM_SUCCESS;
}

/*
 * Called by libtpms when the TPM needs to determine whether physical
 * presence has been asserted
 */
static TPM_RESULT tpm_ltpms_io_getphysicalpresence(TPM_BOOL *physicalPresence,
                                                   uint32_t tpm_number)
{
    *physicalPresence = FALSE;

    return TPM_SUCCESS;
}

struct libtpms_callbacks callbacks = {
    .sizeOfStruct               = sizeof(struct libtpms_callbacks),
    .tpm_nvram_init             = tpm_ltpms_nvram_init,
    .tpm_nvram_loaddata         = tpm_ltpms_nvram_loaddata,
    .tpm_nvram_storedata        = tpm_ltpms_nvram_storedata,
    .tpm_nvram_deletename       = tpm_ltpms_nvram_deletename,
    .tpm_io_init                = tpm_ltpms_io_init,
    .tpm_io_getlocality         = tpm_ltpms_io_getlocality,
    .tpm_io_getphysicalpresence = tpm_ltpms_io_getphysicalpresence,
};

/*****************************************************************/

/*
 * Start the TPM (thread).  If it had been started before, then terminate
 * and start it again.
 */
static int tpm_ltpms_startup_tpm(TPMBackend *tb)
{
    struct TPMLTPMsState *tpm_ltpms = TPM_LIBTPMS(tb);

    /* 'power-reset' a running TPM; if none is running start one */
    tpm_backend_thread_tpm_reset(&tpm_ltpms->tbt, tpm_ltpms_worker_thread,
                                 &tpm_ltpms->tpm_thread_params);

    return 0;
}

static void tpm_ltpms_terminate_tpm_thread(TPMBackend *tb)
{
    struct TPMLTPMsState *tpm_ltpms = TPM_LIBTPMS(tb);

    tpm_backend_thread_end(&tpm_ltpms->tbt);
}

static void tpm_ltpms_reset(TPMBackend *tb)
{
    TPMLTPMsState *tpm_ltpms = TPM_LIBTPMS(tb);

    DPRINTF("tpm_libtpms: Resetting TPM libtpms backend\n");

    tpm_ltpms_terminate_tpm_thread(tb);

    tpm_ltpms->had_fatal_error = false;
}

static int tpm_ltpms_init(TPMBackend *tb, TPMState *s,
                          TPMRecvDataCB *recv_data_cb)
{
    TPMLTPMsState *tpm_ltpms = TPM_LIBTPMS(tb);

    if (TPMLIB_RegisterCallbacks(&callbacks) != TPM_SUCCESS) {
        qerror_report(ERROR_CLASS_GENERIC_ERROR,
                      "TPM libtpms callback registration failed");
        return -1;
    }

    tpm_ltpms->tpm_thread_params.tpm_state = s;
    tpm_ltpms->tpm_thread_params.recv_data_callback = recv_data_cb;
    tpm_ltpms->tpm_thread_params.tb = tb;

    qemu_mutex_init(&tpm_ltpms->tpm_initialized_mutex);

    return 0;
}

static bool tpm_ltpms_get_tpm_established_flag(TPMBackend *tb)
{
    TPMLTPMsState *tpm_ltpms = TPM_LIBTPMS(tb);
    TPM_BOOL tpmEstablished = false;

    qemu_mutex_lock(&tpm_ltpms->tpm_initialized_mutex);
    if (tpm_ltpms->tpm_initialized) {
        TPM_IO_TpmEstablished_Get(&tpmEstablished);
    }
    qemu_mutex_unlock(&tpm_ltpms->tpm_initialized_mutex);

    return tpmEstablished;
}

static bool tpm_ltpms_get_startup_error(TPMBackend *tb)
{
    return false;
}

static size_t tpm_ltpms_realloc_buffer(TPMSizedBuffer *sb)
{
    size_t wanted_size = tpmlib_get_prop(TPMPROP_TPM_BUFFER_MAX);

    if (sb->size != wanted_size) {
        TPM_RESULT res = TPM_Realloc(&sb->buffer, wanted_size);
        if (res == TPM_SUCCESS) {
            sb->size = wanted_size;
        } else {
            qerror_report(ERROR_CLASS_GENERIC_ERROR,
                          "TPM memory allocation failed");
            abort();
        }
    }
    return sb->size;
}

static void tpm_ltpms_deliver_request(TPMBackend *tb)
{
    TPMLTPMsState *tpm_ltpms = TPM_LIBTPMS(tb);

    tpm_backend_thread_deliver_request(&tpm_ltpms->tbt);
}

static void tpm_ltpms_cancel_cmd(TPMBackend *be)
{
}

static const char *tpm_ltpms_create_desc(void)
{
    return "libtpms TPM backend driver";
}

static TPMBackend *tpm_ltpms_create(QemuOpts *opts, const char *id)
{
    Object *obj = object_new(TYPE_TPM_LIBTPMS);
    TPMBackend *tb = TPM_BACKEND(obj);
    const char *value;

    tb->id = g_strdup(id);
    tb->fe_model = -1;
    tb->ops = &tpm_ltpms_driver;

    value = qemu_opt_get(opts, "nvram");
    if (!value) {
        qerror_report(QERR_MISSING_PARAMETER, "nvram");
        goto err_exit;
    }
    tb->nvram_id = g_strdup(value);

    return tb;

err_exit:
    g_free(tb->id);

    return NULL;
}

static void tpm_ltpms_destroy(TPMBackend *tb)
{
    tpm_ltpms_terminate_tpm_thread(tb);

    g_free(tb->id);
    g_free(tb->nvram_id);
}

static const QemuOptDesc tpm_ltpms_cmdline_opts[] = {
    TPM_STANDARD_CMDLINE_OPTS,
    {
        .name = "nvram",
        .type = QEMU_OPT_STRING,
        .help = "NVRAM drive id",
    },
    { /* end of list */ },
};

static const TPMDriverOps tpm_ltpms_driver = {
    .type                     = TPM_TYPE_LIBTPMS,
    .opts                     = tpm_ltpms_cmdline_opts,
    .desc                     = tpm_ltpms_create_desc,
    .create                   = tpm_ltpms_create,
    .destroy                  = tpm_ltpms_destroy,
    .init                     = tpm_ltpms_init,
    .startup_tpm              = tpm_ltpms_startup_tpm,
    .realloc_buffer           = tpm_ltpms_realloc_buffer,
    .reset                    = tpm_ltpms_reset,
    .had_startup_error        = tpm_ltpms_get_startup_error,
    .deliver_request          = tpm_ltpms_deliver_request,
    .cancel_cmd               = tpm_ltpms_cancel_cmd,
    .get_tpm_established_flag = tpm_ltpms_get_tpm_established_flag,
};


static void tpm_ltpms_inst_init(Object *obj)
{
}

static void tpm_ltpms_inst_finalize(Object *obj)
{
}

static void tpm_ltpms_class_init(ObjectClass *klass, void *data)
{
    TPMBackendClass *tbc = TPM_BACKEND_CLASS(klass);

    tbc->ops = &tpm_ltpms_driver;
}

static const TypeInfo tpm_ltpms_info = {
    .name = TYPE_TPM_LIBTPMS,
    .parent = TYPE_TPM_BACKEND,
    .instance_size = sizeof(TPMLTPMsState),
    .class_init = tpm_ltpms_class_init,
    .instance_init = tpm_ltpms_inst_init,
    .instance_finalize = tpm_ltpms_inst_finalize,
};

static void tpm_libtpms_register(void)
{
    type_register_static(&tpm_ltpms_info);
    tpm_register_driver(&tpm_ltpms_driver);
}

type_init(tpm_libtpms_register)
