#include "nameserver.h"
#include "syscall.h"
#include "uart.h"

// Simple string functions
static int ns_strcmp(const char *s1, const char *s2) {
    while (*s1 && (*s1 == *s2)) {
        s1++;
        s2++;
    }
    return *(unsigned char *)s1 - *(unsigned char *)s2;
}

static void ns_strncpy(char *dest, const char *src, int n) {
    int i = 0;
    while (i < n - 1 && src[i]) {
        dest[i] = src[i];
        i++;
    }
    dest[i] = '\0';
}

typedef struct {
    char name[NS_MAX_NAME_LEN];
    int tid;
    int used;
} NsEntry;

static NsEntry ns_entries[NS_MAX_ENTRIES];

static int ns_find_name(const char *name) {
    for (int i = 0; i < NS_MAX_ENTRIES; i++) {
        if (ns_entries[i].used && ns_strcmp(ns_entries[i].name, name) == 0) {
            return i;
        }
    }
    return -1;
}

//assume name from sender with null terminator or at least NS_MAX_NAME_LEN chars
static int ns_add_entry(const char *name, int tid) {
    int idx = ns_find_name(name);
    if (idx >= 0) {
        ns_entries[idx].tid = tid;
        return 0;
    }

    for (int i = 0; i < NS_MAX_ENTRIES; i++) {
        if (!ns_entries[i].used) {
            ns_strncpy(ns_entries[i].name, name, NS_MAX_NAME_LEN);
            ns_entries[i].tid = tid;
            ns_entries[i].used = 1;
            return 0;
        }
    }
    return -1;  
}

void nameserver_task(void) {
    
    for (int i = 0; i < NS_MAX_ENTRIES; i++) {
        ns_entries[i].used = 0;
    }

    int sender_tid;
    NsRequest req;
    NsResponse resp;

    while (1) {
        int len = Receive(&sender_tid, (char *)&req, sizeof(NsRequest));
        if (len < 0) continue;


        resp.status = 0;
        resp.tid = -1;

        switch (req.type) {
            case NS_REGISTER:
                resp.status = ns_add_entry(req.name, req.tid);
                break;

            case NS_WHOIS:
                {
                    int idx = ns_find_name(req.name);
                    if (idx >= 0) {
                        resp.tid = ns_entries[idx].tid;
                        resp.status = 0;
                    } else {
                        resp.status = -1;
                    }
                }
                break;

            default:
                resp.status = -2;
                break;
        }

        Reply(sender_tid, (const char *)&resp, sizeof(NsResponse));
    }
}

int RegisterAs(const char *name) {
    NsRequest req;
    NsResponse resp;

    req.type = NS_REGISTER;
    req.tid = MyTid();
    ns_strncpy(req.name, name, NS_MAX_NAME_LEN);

    int ret = Send(NAMESERVER_TID, (const char *)&req, sizeof(NsRequest),
                   (char *)&resp, sizeof(NsResponse));
    if (ret < 0) return ret;

    return resp.status;
}

int WhoIs(const char *name) {
    NsRequest req;
    NsResponse resp;

    req.type = NS_WHOIS;
    ns_strncpy(req.name, name, NS_MAX_NAME_LEN);

    int ret = Send(NAMESERVER_TID, (const char *)&req, sizeof(NsRequest),
                   (char *)&resp, sizeof(NsResponse));
    if (ret < 0) return ret;

    if (resp.status < 0) return resp.status;
    return resp.tid;
}