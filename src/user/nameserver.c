#include "nameserver.h"
#include "syscall.h"
#include "uart.h"
#include "text_util.h"


// NsEntry: Stores data about an entry in the name server
typedef struct {
    char name[NS_MAX_NAME_LEN];
    int tid;
    int used;
} NsEntry;


// init_ns_entry: Default initializes an entry in the name server
static void init_ns_entry(NsEntry *ns_entry) {
    ns_entry->tid = 0;
    ns_entry->used = 0;
} 


static NsEntry NS_Entries[NS_MAX_ENTRIES];

// ns_find_name: Finds a name within the name server
static int ns_find_name(const char *name) {
    for (int i = 0; i < NS_MAX_ENTRIES; i++) {
        if (NS_Entries[i].used && strcmp(NS_Entries[i].name, name) == 0) {
            return i;
        }
    }
    return -1;
}

//assume name from sender with null terminator or at least NS_MAX_NAME_LEN chars
static int ns_add_entry(const char *name, int tid) {
    int idx = ns_find_name(name);
    if (idx >= 0) {
        NS_Entries[idx].tid = tid;
        return 0;
    }

    for (int i = 0; i < NS_MAX_ENTRIES; i++) {
        if (!NS_Entries[i].used) {
            strncpy(NS_Entries[i].name, name, NS_MAX_NAME_LEN);
            NS_Entries[i].tid = tid;
            NS_Entries[i].used = 1;
            return 0;
        }
    }
    return -1;  
}

void nameserver_task(void) {
    for (int i = 0; i < NS_MAX_ENTRIES; i++) {
        init_ns_entry(&(NS_Entries[i]));
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
                        resp.tid = NS_Entries[idx].tid;
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
    strncpy(req.name, name, NS_MAX_NAME_LEN);

    int ret = Send(NAMESERVER_TID, (const char *)&req, sizeof(NsRequest),
                   (char *)&resp, sizeof(NsResponse));
    if (ret < 0) return ret;

    return resp.status;
}

int WhoIs(const char *name) {
    NsRequest req;
    NsResponse resp;

    req.type = NS_WHOIS;
    strncpy(req.name, name, NS_MAX_NAME_LEN);

    int ret = Send(NAMESERVER_TID, (const char *)&req, sizeof(NsRequest),
                   (char *)&resp, sizeof(NsResponse));
    if (ret < 0) return ret;

    if (resp.status < 0) return resp.status;
    return resp.tid;
}