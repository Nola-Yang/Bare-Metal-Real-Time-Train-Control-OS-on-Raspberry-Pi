#include "performance_test.h"
#include "syscall.h"
#include "uart.h"
#include "timer.h"
#include "task_scheduler.h"
#include "util.h"


#define TEST_COUNT 5000
#define MSG_TYPE_COUNT 3
#define MAX_MSG_LEN 256
#define TIMER_MSG_LEN 12


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
static void print_csv_row(char first_drive, int msg_len, uint64_t time) {
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

// send_first_func: Sending task for the "send first" execution order
static void send_first_func() {
	uint64_t avg_time = 0;
	uint32_t time = 0;

	int msg_len = 0;
	int reply_len = 0;
	char *msg;
	char reply[MAX_MSG_LEN] = {0};

	for (uint32_t i = 0; i < MSG_TYPE_COUNT; ++i) {
		avg_time = 0;
		msg_len = Msg_Lens[i];
		reply_len = msg_len;
		msg = Msgs[i];

		for (uint32_t j = 0; j < TEST_COUNT; ++j) {
			Yield();

			time = read_timer();
			Send(RECV_AFT_TASK_ID, msg, msg_len, reply, reply_len);
			time = read_timer() - time;

			avg_time += time;
		}

		avg_time /= TEST_COUNT;
		print_csv_row(Sender_First, msg_len, avg_time);
	}

	Exit();
}

// recv_after_func: Receiving task for the "send first" execution order
static void recv_after_func() {
	int send_task_tid = 0;
	int msg_len = 0;
	int reply_len = 0;
	char msg[MAX_MSG_LEN];
	char *reply;

	for (uint32_t i = 0; i < MSG_TYPE_COUNT; ++i) {
		msg_len = Msg_Lens[i];
		reply_len = msg_len;
		reply = Msgs[i];

		for (uint32_t j = 0; j < TEST_COUNT; ++j) {
			Yield();

			Receive(&send_task_tid, msg, msg_len);
			Reply(SEND_FIRST_TASK_ID, reply, reply_len);
		}
	}

	Exit();
}

// recv_first_func: Receving task for the "receive first" execution order
static void recv_first_func() {
	int send_task_tid = 0;
	int msg_len = 0;
	int reply_len = 0;
	char msg[MAX_MSG_LEN];
	char *reply;
	char *timer_msg;
	char timer_msg_first_char;

	uint32_t start_time = 0;
	uint32_t end_time = 0;
	uint64_t avg_time = 0;

	for (uint32_t i = 0; i < MSG_TYPE_COUNT; ++i) {
		avg_time = 0;
		msg_len = Msg_Lens[i];
		reply_len = msg_len;
		reply = Msgs[i];

		for (uint32_t j = 0; j < TEST_COUNT; ++j) {
			Yield();

			start_time = read_timer();
			Receive(&send_task_tid, msg, msg_len);
			Reply(SEND_AFT_TASK_ID, reply, reply_len);

			// get back the end time from the sender
			Receive(&send_task_tid, timer_msg, TIMER_MSG_LEN);
			Reply(SEND_AFT_TASK_ID, "acked", TIMER_MSG_LEN);
			
			timer_msg_first_char = timer_msg[0];
			timer_msg++;
			a2ui(timer_msg_first_char, &timer_msg, 10, &end_time);

			avg_time += end_time - start_time;
		}

		avg_time /= TEST_COUNT;
		print_csv_row(Receiver_First, msg_len, avg_time);
	}

	Exit();
}

// send_after_func: Sending task for the "receive first" execution order
static void send_after_func() {
	uint32_t end_time = 0;
	char end_time_msg[TIMER_MSG_LEN];
	char end_time_ack_msg[TIMER_MSG_LEN];

	int msg_len = 0;
	int reply_len = 0;
	char *msg;
	char reply[MAX_MSG_LEN] = {0};

	for (uint32_t i = 0; i < MSG_TYPE_COUNT; ++i) {
		msg_len = Msg_Lens[i];
		reply_len = msg_len;
		msg = Msgs[i];

		for (uint32_t j = 0; j < TEST_COUNT; ++j) {
			Yield();

			Send(RECV_FIRST_TASK_ID, msg, msg_len, reply, reply_len);
			end_time = read_timer();

			// send the end time
			ui2a(end_time, 10, end_time_msg);
			Send(RECV_FIRST_TASK_ID, end_time_msg, TIMER_MSG_LEN, end_time_ack_msg, TIMER_MSG_LEN);
			Yield();
		}
	}

	Exit();
}

// perform_test_task: Task for the overall performance test
static void perform_test_task() {
	Create(RPS_CLIENT_PRIORITY, send_first_func);
	Create(RPS_CLIENT_PRIORITY, recv_after_func);

	int recv_first_priority = RPS_CLIENT_PRIORITY - 1;

	Create(recv_first_priority, recv_first_func);
	Create(recv_first_priority, send_after_func);
	Exit();
}

void perform_test_run() {
	print_test_spec();
	Create(RPS_SERVER_PRIORITY, perform_test_task);
}