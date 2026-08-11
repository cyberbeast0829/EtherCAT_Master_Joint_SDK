#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esi_parser.h"

#define ESI_TAG_MAX 64
#define ESI_VAL_MAX 256
#define ESI_MAX_PDO_ENTRIES 32
#define ESI_MAX_SMS 8

typedef struct {
    uint16_t index;
    uint8_t  subindex;
    uint8_t  bitlen;
} esi_pdo_entry_t;

typedef struct {
    unsigned int count;
    esi_pdo_entry_t entries[ESI_MAX_PDO_ENTRIES];
    uint16_t pdo_index;   /* 0x1600 / 0x1A00 */
    int sm;               /* SM index for this PDO */
} esi_pdo_t;

typedef struct {
    uint32_t vendor_id;
    uint32_t product_code;
    uint32_t revision_no;
    uint16_t dc_assign_activate;
    int sm_numbers[ESI_MAX_SMS];  /* process-data SM indices */
    int sm_count;
    esi_pdo_t rx_pdo;             /* RxPDO: SM2 Outputs */
    esi_pdo_t tx_pdo;             /* TxPDO: SM3 Inputs  */
} esi_ctx_t;

/* helpers */
static uint32_t parse_hex(const char *s)
{
    const char *p = s ? s : "";
    while (*p == '#' || *p == 'x' || *p == 'X') p++;
    return (uint32_t)strtoul(p, NULL, 16);
}

static uint32_t parse_attr_hex(const char *line, const char *attr)
{
    char key[128], eq[4], val[128];
    const char *p = line;
    const char *found;

    snprintf(key, sizeof(key), "%s=\"", attr);
    found = strstr(p, key);
    if (!found) {
        snprintf(key, sizeof(key), "%s='", attr);
        found = strstr(p, key);
    }
    if (!found) return 0;

    p = found + strlen(key);
    {
        const char *end = strchr(p, '"');
        if (!end) end = strchr(p, '\'');
        if (!end) return 0;

        size_t n = (size_t)(end - p);
        if (n >= sizeof(val)) n = sizeof(val) - 1;
        memcpy(val, p, n);
        val[n] = '\0';
    }

    return parse_hex(val);
}

static const char *skip_whitespace(const char *p)
{
    while (p && *p && isspace((unsigned char)*p)) p++;
    return p;
}

static int extract_content(const char *buf, size_t buf_sz,
        const char *tag, char *out, size_t out_sz)
{
    char open[ESI_TAG_MAX + 4], close[ESI_TAG_MAX + 4];
    const char *start, *end;

    snprintf(open, sizeof(open), "<%s", tag);
    snprintf(close, sizeof(close), "</%s>", tag);

    start = strstr(buf, open);
    if (!start) return -1;

    start = strchr(start, '>');
    if (!start) return -1;
    start++;

    end = strstr(start, close);
    if (!end) return -1;

    {
        size_t n = (size_t)(end - start);
        if (n >= out_sz) n = out_sz - 1;
        memcpy(out, start, n);
        out[n] = '\0';
    }
    return 0;
}

static int extract_tag_attr_val(const char *buf, size_t buf_sz,
        const char *tag, const char *attr, char *out, size_t out_sz)
{
    char open[ESI_TAG_MAX + 4];
    const char *start, *end;
    char attr_eq[ESI_TAG_MAX + 4];

    snprintf(open, sizeof(open), "<%s", tag);
    snprintf(attr_eq, sizeof(attr_eq), "%s=\"", attr);
    start = strstr(buf, open);
    if (!start) return -1;

    end = strstr(start, attr_eq);
    if (!end) {
        snprintf(attr_eq, sizeof(attr_eq), "%s='", attr);
        end = strstr(start, attr_eq);
    }
    if (!end) return -1;

    end += strlen(attr_eq);
    start = end;
    end = strchr(start, '"');
    if (!end) end = strchr(start, '\'');
    if (!end) return -1;

    {
        size_t n = (size_t)(end - start);
        if (n >= out_sz) n = out_sz - 1;
        memcpy(out, start, n);
        out[n] = '\0';
    }
    return 0;
}

static int read_text_file(const char *path, char **buf, size_t *sz)
{
    FILE *fp;
    long len;

    fp = fopen(path, "rb");
    if (!fp) return -1;

    fseek(fp, 0, SEEK_END);
    len = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    if (len <= 0) { fclose(fp); return -1; }

    *buf = malloc((size_t)len + 1);
    if (!*buf) { fclose(fp); return -1; }

    *sz = fread(*buf, 1, (size_t)len, fp);
    fclose(fp);

    (*buf)[*sz] = '\0';
    return 0;
}

static const char *next_tag(const char *p, const char *end,
        const char *tag, const char **tag_end)
{
    char open[ESI_TAG_MAX + 2];
    const char *found;

    snprintf(open, sizeof(open), "<%s", tag);
    found = NULL;

    while (p < end) {
        const char *candidate = strstr(p, open);
        if (!candidate) break;

        const char *close_bracket = strchr(candidate, '>');
        if (!close_bracket) break;

        /* Check if it's self-closing or opening */
        if (close_bracket > candidate && *(close_bracket - 1) == '/') {
            *tag_end = close_bracket + 1;
            return candidate;
        }

        /* It's an opening tag - good */
        *tag_end = close_bracket + 1;
        return candidate;

        p = close_bracket + 1;
    }

    /* Try self-closing form */
    found = strstr(p, open);
    if (!found) {
        snprintf(open, sizeof(open), "<%s ", tag);
        found = strstr(p, open);
    }
    if (!found) return NULL;

    *tag_end = strchr(found, '>');
    if (!*tag_end) return NULL;
    *tag_end = *tag_end + 1;

    return found;
}

/* ---- main parse ---- */

static int parse_sm(esi_ctx_t *ctx, const char *start, const char *end)
{
    char content[256] = {0};
    const char *gt = memchr(start, '>', (size_t)(end - start));

    if (gt && (size_t)(end - (gt + 1)) < sizeof(content)) {
        memcpy(content, gt + 1, (size_t)(end - (gt + 1)));
        content[(size_t)(end - (gt + 1))] = '\0';
    }

    {
        char *trim = content;
        while (*trim && isspace((unsigned char)*trim)) trim++;
        {
            char *e = trim + strlen(trim) - 1;
            while (e >= trim && isspace((unsigned char)*e)) *e-- = '\0';
        }

        if (!strcmp(trim, "MBoxOut") || !strcmp(trim, "MBoxIn"))
            return 0;
    }

    if (ctx->sm_count >= ESI_MAX_SMS) return -1;

    {
        char enable_str[8] = "";
        extract_tag_attr_val(start, (size_t)(end - start),
                "Sm", "Enable", enable_str, sizeof(enable_str));

        /* Only collect enabled process-data SMs */
        if (enable_str[0] != '1') return 0;
    }

    ctx->sm_numbers[ctx->sm_count] =
        (int)parse_attr_hex(start, "StartAddress"); /* sm number proxy */
    /* We use the position in the SM list as the index */
    ctx->sm_count++;
    return 0;
}

static int parse_rxpdo(esi_ctx_t *ctx, const char *start, const char *end)
{
    /* 只解析第一组 RxPdo；后续的 RxPdo（如 0x160D/0x160F/0x1611 备选映射）忽略 */
    if (ctx->rx_pdo.count) return 0;

    char sm_str[8] = "";
    extract_tag_attr_val(start, (size_t)(end - start),
            "RxPdo", "Sm", sm_str, sizeof(sm_str));
    ctx->rx_pdo.sm = atoi(sm_str);

    {
        char idx[32] = "";
        if (extract_content(start, (size_t)(end - start),
                    "Index", idx, sizeof(idx)) != 0) {
            extract_tag_attr_val(start, (size_t)(end - start),
                    "Index", "DependOnSlot", idx, sizeof(idx));
        }
        ctx->rx_pdo.pdo_index = (uint16_t)parse_hex(idx);
    }

    {
        const char *pos = start;
        while (pos < end) {
            const char *entry_start, *entry_end = NULL;
            char close[32];

            entry_start = strstr(pos, "<Entry>");
            if (!entry_start || entry_start >= end) break;

            snprintf(close, sizeof(close), "</Entry>");
            entry_end = strstr(entry_start, close);
            if (!entry_end || entry_end >= end) break;
            entry_end += strlen(close);

            {
                char idx[32] = "", si[32] = "", bl[32] = "";
                if (extract_content(entry_start, (size_t)(entry_end - entry_start),
                            "Index", idx, sizeof(idx)) != 0) {
                    extract_tag_attr_val(entry_start,
                            (size_t)(entry_end - entry_start),
                            "Index", "DependOnSlot", idx, sizeof(idx));
                }
                extract_content(entry_start, (size_t)(entry_end - entry_start),
                        "SubIndex", si, sizeof(si));
                extract_content(entry_start, (size_t)(entry_end - entry_start),
                        "BitLen", bl, sizeof(bl));

                if (idx[0] && bl[0] &&
                        ctx->rx_pdo.count < ESI_MAX_PDO_ENTRIES) {
                    esi_pdo_entry_t *e =
                        &ctx->rx_pdo.entries[ctx->rx_pdo.count++];
                    e->index = (uint16_t)parse_hex(idx);
                    e->subindex = (uint8_t)atoi(si);
                    e->bitlen = (uint8_t)atoi(bl);
                }
            }

            pos = entry_end;
        }
    }
    return 0;
}

static int parse_txpdo(esi_ctx_t *ctx, const char *start, const char *end)
{
    if (ctx->tx_pdo.count) return 0;

    char sm_str[8] = "";
    extract_tag_attr_val(start, (size_t)(end - start),
            "TxPdo", "Sm", sm_str, sizeof(sm_str));
    ctx->tx_pdo.sm = atoi(sm_str);

    {
        char idx[32] = "";
        if (extract_content(start, (size_t)(end - start),
                    "Index", idx, sizeof(idx)) != 0) {
            extract_tag_attr_val(start, (size_t)(end - start),
                    "Index", "DependOnSlot", idx, sizeof(idx));
        }
        ctx->tx_pdo.pdo_index = (uint16_t)parse_hex(idx);
    }

    {
        const char *pos = start;
        while (pos < end) {
            const char *entry_start, *entry_end = NULL;
            char close[32];

            entry_start = strstr(pos, "<Entry>");
            if (!entry_start || entry_start >= end) break;

            snprintf(close, sizeof(close), "</Entry>");
            entry_end = strstr(entry_start, close);
            if (!entry_end || entry_end >= end) break;
            entry_end += strlen(close);

            {
                char idx[32] = "", si[32] = "", bl[32] = "";
                if (extract_content(entry_start,
                            (size_t)(entry_end - entry_start),
                            "Index", idx, sizeof(idx)) != 0) {
                    extract_tag_attr_val(entry_start,
                            (size_t)(entry_end - entry_start),
                            "Index", "DependOnSlot", idx, sizeof(idx));
                }
                extract_content(entry_start,
                        (size_t)(entry_end - entry_start),
                        "SubIndex", si, sizeof(si));
                extract_content(entry_start,
                        (size_t)(entry_end - entry_start),
                        "BitLen", bl, sizeof(bl));

                if (idx[0] && bl[0] &&
                        ctx->tx_pdo.count < ESI_MAX_PDO_ENTRIES) {
                    esi_pdo_entry_t *e =
                        &ctx->tx_pdo.entries[ctx->tx_pdo.count++];
                    e->index = (uint16_t)parse_hex(idx);
                    e->subindex = (uint8_t)atoi(si);
                    e->bitlen = (uint8_t)atoi(bl);
                }
            }

            pos = entry_end;
        }
    }
    return 0;
}

static int parse_dc(esi_ctx_t *ctx, const char *dc_start, const char *dc_end)
{
    const char *pos = dc_start;

    while (pos < dc_end) {
        const char *mode_start = strstr(pos, "<OpMode>");
        const char *mode_end;
        if (!mode_start || mode_start >= dc_end) break;

        mode_end = strstr(mode_start, "</OpMode>");
        if (!mode_end || mode_end >= dc_end) break;
        mode_end += strlen("</OpMode>");

        {
            char name[128] = "";
            extract_content(mode_start,
                    (size_t)(mode_end - mode_start),
                    "Name", name, sizeof(name));
            if (!strcmp(name, "DC")) {
                char aa[32] = "";
                extract_content(mode_start,
                        (size_t)(mode_end - mode_start),
                        "AssignActivate", aa, sizeof(aa));
                ctx->dc_assign_activate = (uint16_t)parse_hex(aa);
            }
        }

        pos = mode_end;
    }
    return 0;
}

const jsdk_joint_profile_t *esi_profile_load(
        const char *esi_path,
        uint32_t vendor_id,
        uint32_t product_code)
{
    char *xml = NULL;
    size_t xml_sz = 0;
    esi_ctx_t ctx;
    jsdk_joint_profile_t *profile = NULL;
    uint8_t *heap = NULL;

    memset(&ctx, 0, sizeof(ctx));

    if (read_text_file(esi_path, &xml, &xml_sz) != 0)
        goto fail;

    /* Vendor ID from <Vendor><Id> */
    {
        char vendor_block[512] = "";
        if (extract_content(xml, xml_sz,
                    "Vendor", vendor_block, sizeof(vendor_block)) == 0) {
            char vid[128] = "";
            if (extract_content(vendor_block, strlen(vendor_block),
                        "Id", vid, sizeof(vid)) == 0) {
                ctx.vendor_id = parse_hex(vid);
            }
        }
    }

    /* Device identity */
    {
        const char *dev = strstr(xml, "<Device ");
        if (dev) {
            ctx.product_code = parse_attr_hex(dev, "ProductCode");
            ctx.revision_no  = parse_attr_hex(dev, "RevisionNo");
        }
    }

    /* Override / validate */
    if (vendor_id && ctx.vendor_id && vendor_id != ctx.vendor_id)
        goto fail;
    if (product_code && ctx.product_code && product_code != ctx.product_code)
        goto fail;

    /* SM elements between <Device> and </Device> */
    {
        const char *dev_start = strstr(xml, "<Device");
        const char *dev_end   = strstr(xml, "</Device>");
        if (dev_start && dev_end) {
            const char *pos = dev_start;
            while (pos < dev_end) {
                const char *sm_start = strstr(pos, "<Sm ");
                const char *sm_end;
                if (!sm_start || sm_start >= dev_end) break;

                sm_end = strstr(sm_start, ">");
                if (!sm_end) break;
                /* Find matching </Sm> or self-closing /> */
                if (*(sm_end - 1) == '/') {
                    sm_end++;
                } else {
                    sm_end = strstr(sm_start, "</Sm>");
                    if (!sm_end) break;
                    sm_end += strlen("</Sm>");
                }

                parse_sm(&ctx, sm_start, sm_end);
                pos = sm_end;
            }
        }
    }

    /* DC */
    {
        const char *dc_start = strstr(xml, "<Dc>");
        const char *dc_end;
        if (dc_start) {
            dc_end = strstr(dc_start, "</Dc>");
            if (dc_end) {
                dc_end += strlen("</Dc>");
                parse_dc(&ctx, dc_start, dc_end);
            }
        }
    }

    /* RxPdo / TxPdo: search in both <Device> and <Module> blocks */
    {
        const char *pos = xml;
        while (pos < xml + xml_sz) {
            const char *pdo_start, *pdo_end = NULL;

            pdo_start = strstr(pos, "<RxPdo ");
            if (pdo_start) {
                pdo_end = strstr(pdo_start, "</RxPdo>");
                if (!pdo_end) break;
                pdo_end += strlen("</RxPdo>");
                parse_rxpdo(&ctx, pdo_start, pdo_end);
                pos = pdo_end;
                continue;
            }

            pdo_start = strstr(pos, "<TxPdo ");
            if (pdo_start) {
                pdo_end = strstr(pdo_start, "</TxPdo>");
                if (!pdo_end) break;
                pdo_end += strlen("</TxPdo>");
                parse_txpdo(&ctx, pdo_start, pdo_end);
                pos = pdo_end;
                continue;
            }

            break;
        }
    }

    if (!ctx.rx_pdo.count || !ctx.tx_pdo.count)
        goto fail;

    /* Allocate one contiguous block:
     *   jsdk_joint_profile_t + syncs[5] + rx_pdo_info[1] + tx_pdo_info[1]
     *   + rx_entries[n] + tx_entries[n]
     */
    {
        size_t alloc_sz = sizeof(jsdk_joint_profile_t)
            + 5 * sizeof(ec_sync_info_t)
            + sizeof(ec_pdo_info_t) + sizeof(ec_pdo_info_t)
            + ctx.rx_pdo.count * sizeof(ec_pdo_entry_info_t)
            + ctx.tx_pdo.count * sizeof(ec_pdo_entry_info_t);
        void *blob = calloc(1, alloc_sz);

        if (!blob) goto fail;

        profile = (jsdk_joint_profile_t *)blob;
        {
            ec_sync_info_t *syncs = (ec_sync_info_t *)(profile + 1);
            ec_pdo_info_t  *rx_pdi = (ec_pdo_info_t *)(syncs + 5);
            ec_pdo_info_t  *tx_pdi = rx_pdi + 1;
            ec_pdo_entry_info_t *rx_ent = (ec_pdo_entry_info_t *)(tx_pdi + 1);
            ec_pdo_entry_info_t *tx_ent = rx_ent + ctx.rx_pdo.count;
            unsigned int i;

            /* Fill syncs: SM0/1 = mailbox disabled, SM2 = rx, SM3 = tx */
            syncs[0].index = 0;
            syncs[0].dir = EC_DIR_OUTPUT;
            syncs[0].n_pdos = 0;
            syncs[0].pdos = NULL;
            syncs[0].watchdog_mode = EC_WD_DISABLE;

            syncs[1].index = 1;
            syncs[1].dir = EC_DIR_INPUT;
            syncs[1].n_pdos = 0;
            syncs[1].pdos = NULL;
            syncs[1].watchdog_mode = EC_WD_DISABLE;

            syncs[2].index = 2;
            syncs[2].dir = EC_DIR_OUTPUT;
            syncs[2].n_pdos = 1;
            syncs[2].pdos = rx_pdi;
            syncs[2].watchdog_mode = EC_WD_ENABLE;

            syncs[3].index = 3;
            syncs[3].dir = EC_DIR_INPUT;
            syncs[3].n_pdos = 1;
            syncs[3].pdos = tx_pdi;
            syncs[3].watchdog_mode = EC_WD_DISABLE;

            syncs[4].index = 0xff;

            /* RxPDO info */
            rx_pdi->index = ctx.rx_pdo.pdo_index;
            rx_pdi->n_entries = ctx.rx_pdo.count;
            rx_pdi->entries = rx_ent;

            /* TxPDO info */
            tx_pdi->index = ctx.tx_pdo.pdo_index;
            tx_pdi->n_entries = ctx.tx_pdo.count;
            tx_pdi->entries = tx_ent;

            /* Rx entries */
            for (i = 0; i < ctx.rx_pdo.count; i++) {
                rx_ent[i].index = ctx.rx_pdo.entries[i].index;
                rx_ent[i].subindex = ctx.rx_pdo.entries[i].subindex;
                rx_ent[i].bit_length = ctx.rx_pdo.entries[i].bitlen;
            }

            /* Tx entries */
            for (i = 0; i < ctx.tx_pdo.count; i++) {
                tx_ent[i].index = ctx.tx_pdo.entries[i].index;
                tx_ent[i].subindex = ctx.tx_pdo.entries[i].subindex;
                tx_ent[i].bit_length = ctx.tx_pdo.entries[i].bitlen;
            }

            profile->syncs         = syncs;
            profile->rx_entries    = rx_ent;
            profile->rx_entry_count = ctx.rx_pdo.count;
            profile->tx_entries    = tx_ent;
            profile->tx_entry_count = ctx.tx_pdo.count;
        }

        profile->name      = "ESI-loaded";
        profile->vendor_id = ctx.vendor_id;
        profile->product_code = ctx.product_code;
        profile->revision_no  = ctx.revision_no;
        profile->dc_assign_activate = ctx.dc_assign_activate;
        profile->min_cycle_ns = 50000u;
        profile->mode_csp  = JSDK_MODE_CSP;
        profile->mode_csv  = JSDK_MODE_CSV;
        profile->mode_cst  = JSDK_MODE_CST;
        profile->heap_block = blob;
    }

    free(xml);
    return profile;

fail:
    free(xml);
    return NULL;
}

void jsdk_profile_destroy(const jsdk_joint_profile_t *profile)
{
    if (profile && profile->heap_block) {
        free(profile->heap_block);
    }
}
