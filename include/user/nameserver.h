#ifndef NAMESERVER_H
#define NAMESERVER_H

#define NAMESERVER_TID 1

#define NS_REGISTER 1
#define NS_WHOIS    2

#define NS_MAX_NAME_LEN 32
#define NS_MAX_ENTRIES  64

//request For RegisterAs
typedef struct {
    int type;
    char name[NS_MAX_NAME_LEN];
    int tid;  
} NsRequest;

//response For WhoIs
typedef struct {
    int status;  // 0 = success, negative = error
    int tid;     
} NsResponse;

// Register a name with the name server
// Returns 0 on success, negative on error
int RegisterAs(const char *name);

// Look up a task by name
// Returns tid on success, negative on error
int WhoIs(const char *name);

void nameserver_task(void);

#endif /* NAMESERVER_H */