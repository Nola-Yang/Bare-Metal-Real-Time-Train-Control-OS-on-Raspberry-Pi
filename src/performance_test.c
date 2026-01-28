#include "performance_test.h"
#include "syscall.h"
#include "uart.h"
#include "timer.h"
#include "task_scheduler.h"
#include "util.h"


#define TEST_COUNT 5000
#define MSG_TYPE_COUNT 3
#define MAX_MSG_LEN 256
#define SYNC_MSG_LEN 4


static const char Sender_First = 'S';
static const char Receiver_First = 'R';

static const int Msg_Lens[MSG_TYPE_COUNT] = {4, 64, MAX_MSG_LEN};
static char *Msgs[] = {
	"AAA",
	"BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB",
	"CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC",
};


// print_test_spec: Prints the specified setup based from the Makefile
static void print_test_spec() {
    #ifdef OPT
    uart_printf(CONSOLE, "OPT: enabled\r\n");
    #else
    uart_printf(CONSOLE, "OPT: disabled\r\n");
    #endif

    #ifdef DCACHE
	uart_printf(CONSOLE, "D cache: enabled\r\n");
	#else
	uart_printf(CONSOLE, "D cache: disabled\r\n");
	#endif

	#ifdef ICACHE
	uart_printf(CONSOLE, "I cache: enabled\r\n");
	#else
	uart_printf(CONSOLE, "I cache: disabled\r\n");
	#endif

	uart_printf(CONSOLE, "\r\n");
}

// print_csv_row: Prints a row for the CSV file
static void print_csv_row(char first_drive, int msg_len, uint32_t time) {
	#ifdef OPT
	char *opt = "opt";
	#else
	char *opt = "noopt";
	#endif

	#if defined(DCACHE) && defined(ICACHE)
	char *cache = "bcache";
	#elif defined(DCACHE)
	char *cache = "dcache";
	#elif defined(ICACHE)
	char *cache = "icache";
	#else
	char *cache = "nocache";
	#endif

	uart_printf(CONSOLE, "%s,%s,%c,%d,%d\r\n", opt, cache, first_drive, msg_len, time);
}

// task_a_func: Task A (TID 3)
// Phase 1 (Receive First)
// Phase 2 (Send First)
static void task_a_func() {
	int sender_tid = 0;
	int msg_len = 0;
	int reply_len = 0;
	char msg[MAX_MSG_LEN];
	char *reply;
	char sync_reply[SYNC_MSG_LEN];

	uint64_t avg_time = 0;
	uint32_t time = 0;

	for (uint32_t i = 0; i < MSG_TYPE_COUNT; ++i) {
		msg_len = Msg_Lens[i];
		reply_len = msg_len;
		reply = Msgs[i];

		for (uint32_t j = 0; j < TEST_COUNT; ++j) {
			Receive(&sender_tid, msg, msg_len);
			Reply(sender_tid, reply, reply_len);
		}
	}

	// Sync
	Send(TASK_B_TID, "sync", SYNC_MSG_LEN, sync_reply, SYNC_MSG_LEN);

	//Send First 
	char reply_buf[MAX_MSG_LEN] = {0};

	for (uint32_t i = 0; i < MSG_TYPE_COUNT; ++i) {
		avg_time = 0;
		msg_len = Msg_Lens[i];
		reply_len = msg_len;
		reply = Msgs[i];

		for (uint32_t j = 0; j < TEST_COUNT; ++j) {
			time = read_timer();
			Send(TASK_B_TID, reply, msg_len, reply_buf, reply_len);
			time = read_timer() - time;

			avg_time += time;
		}

		avg_time /= TEST_COUNT;
		print_csv_row(Sender_First, msg_len, avg_time);
	}

	// Sync
	Send(TASK_B_TID, "done", SYNC_MSG_LEN, sync_reply, SYNC_MSG_LEN);

	Exit();
}

// Phase 1 (Receive First)
// Phase 2 (Send First)
static void task_b_func() {
	int sender_tid = 0;
	int msg_len = 0;
	int reply_len = 0;
	char *msg;
	char reply[MAX_MSG_LEN] = {0};
	char sync_msg[SYNC_MSG_LEN];
	char sync_reply[SYNC_MSG_LEN] = "ack";

	uint64_t avg_time = 0;
	uint32_t time = 0;

	for (uint32_t i = 0; i < MSG_TYPE_COUNT; ++i) {
		avg_time = 0;
		msg_len = Msg_Lens[i];
		reply_len = msg_len;
		msg = Msgs[i];

		for (uint32_t j = 0; j < TEST_COUNT; ++j) {
			time = read_timer();
			Send(TASK_A_TID, msg, msg_len, reply, reply_len);
			time = read_timer() - time;

			avg_time += time;
		}

		avg_time /= TEST_COUNT;
		print_csv_row(Receiver_First, msg_len, avg_time);
	}

	// Sync
	Receive(&sender_tid, sync_msg, SYNC_MSG_LEN);
	Reply(sender_tid, sync_reply, SYNC_MSG_LEN);

	//Send First 
	char msg_buf[MAX_MSG_LEN];

	for (uint32_t i = 0; i < MSG_TYPE_COUNT; ++i) {
		msg_len = Msg_Lens[i];
		reply_len = msg_len;
		msg = Msgs[i];

		for (uint32_t j = 0; j < TEST_COUNT; ++j) {
			Receive(&sender_tid, msg_buf, msg_len);
			Reply(sender_tid, msg, reply_len);
		}
	}

	// Sync
	Receive(&sender_tid, sync_msg, SYNC_MSG_LEN);
	Reply(sender_tid, sync_reply, SYNC_MSG_LEN);

	Exit();
}
void perform_test_run() {
	print_test_spec();

	int tid_a = Create(RPS_CLIENT_PRIORITY + 1, task_a_func);
	if (tid_a != TASK_A_TID) {
		uart_printf(CONSOLE, "ERROR: Task A tid=%d (expected %d)\r\n", tid_a, TASK_A_TID);
		Exit();
	}

	int tid_b = Create(RPS_CLIENT_PRIORITY, task_b_func);
	if (tid_b != TASK_B_TID) {
		uart_printf(CONSOLE, "ERROR: Task B tid=%d (expected %d)\r\n", tid_b, TASK_B_TID);
		Exit();
	}
}