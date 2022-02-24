@###############################################
@#
@# EmPy template for generating microRTPS_agent.cpp file
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - msgs (List) list of all msg files
@#  - multi_topics (List) list of all multi-topic names
@#  - ids (List) list of all RTPS msg ids
@###############################################
@{
import genmsg.msgs

from px_generate_uorb_topic_helper import * # this is in Tools/
from px_generate_uorb_topic_files import MsgScope # this is in Tools/

send_topics = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.SEND]
recv_topics = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.RECEIVE]
}@
/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 * Copyright (c) 2018-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <thread>
#include <atomic>
#include <unistd.h>
#include <poll.h>
#include <chrono>
#include <ctime>
#include <csignal>
#include <termios.h>
#include <condition_variable>
#include <queue>

#include <fastcdr/Cdr.h>
#include <fastcdr/FastCdr.h>
#include <fastcdr/exceptions/Exception.h>
#include <fastrtps/Domain.h>

#include "microRTPS_transport.h"
#include "microRTPS_timesync.h"
#include "RtpsTopics.h"

/* Options default values */
#define DEVICE "/dev/ttyACM0"
#define SLEEP_US 1
#define BAUDRATE 460800
#define POLL_MS 1
#define WAIT_CNST 2
#define DEFAULT_RECV_PORT 2020
#define DEFAULT_SEND_PORT 2019
#define DEFAULT_IP "127.0.0.1"

using namespace eprosima;
using namespace eprosima::fastrtps;

/* Global application state */
volatile sig_atomic_t running = 1;
std::unique_ptr<Transport_node> transport_node = nullptr;
std::unique_ptr<RtpsTopics> topics = nullptr;
unsigned long int total_sent = 0, sent = 0;
bool localhost_only = false; // TODO add to RtpsTopics

/* Startup options */
struct options {
	enum class eTransports {
		UART,
		UDP
	};
	eTransports transport = options::eTransports::UART;
	char device[64] = DEVICE;
	int sleep_us = SLEEP_US;
	uint32_t baudrate = BAUDRATE;
	int poll_ms = POLL_MS;
	uint16_t recv_port = DEFAULT_RECV_PORT;
	uint16_t send_port = DEFAULT_SEND_PORT;
	char ip[16] = DEFAULT_IP;
	bool sw_flow_control = false;
	bool hw_flow_control = false;
	bool verbose_debug = false;
	std::string ns = "";
} _options;

/* Prints usage information. */
static void usage(const char *name)
{
	printf("usage: %s [options]\n\n"
	       "  -b <baudrate>           UART device baudrate (default: 460800 baud)\n"
	       "  -d <device>             UART device (default: /dev/ttyACM0)\n"
	       "  -f <sw flow control>    Activates UART link SW flow control\n"
	       "  -h <hw flow control>    Activates UART link HW flow control\n"
	       "  -i <ip_address>         Target IP for UDP (default: 127.0.0.1)\n"
         "  -l                      Use only the local loopback interface for DDS communications\n"
	       "  -n <namespace>          ROS 2 topics namespace\n"
	       "  -p <poll_ms>            Time in ms to poll over UART (default: 1 ms)\n"
	       "  -r <reception port>     UDP inbound port (default: 2020)\n"
	       "  -s <sending port>       UDP outbound port (default: 2019)\n"
	       "  -t <transport>          UDP or UART transport (default: UART)\n"
	       "  -v <debug verbosity>    Add more verbosity to logs\n"
	       "  -w <sleep_time_us>      Time in us for which to sleep in each iteration (default: 1 ms)\n",
	       name);
}

/* Parses startup options. */
static int parse_options(int argc, char **argv)
{
	int ch;

	while ((ch = getopt(argc, argv, "t:d:w:b:p:r:s:i:fhvln:")) != EOF) {
		switch (ch) {
		case 't': _options.transport      = strcmp(optarg, "UDP") == 0 ?
							    options::eTransports::UDP
							    : options::eTransports::UART;    break;

		case 'd': if (nullptr != optarg) strcpy(_options.device, optarg);   break;

		case 'w': _options.sleep_us        = strtol(optarg, nullptr, 10);   break;

		case 'b': _options.baudrate        = strtoul(optarg, nullptr, 10);  break;

		case 'p': _options.poll_ms         = strtol(optarg, nullptr, 10);   break;

		case 'r': _options.recv_port       = strtoul(optarg, nullptr, 10);  break;

		case 's': _options.send_port       = strtoul(optarg, nullptr, 10);  break;

		case 'i': if (nullptr != optarg) strcpy(_options.ip, optarg);       break;

		case 'f': _options.sw_flow_control = true;                          break;

		case 'h': _options.hw_flow_control = true;                          break;

		case 'v': _options.verbose_debug = true;                            break;

    case 'l': localhost_only = true;                                    break;

		case 'n': if (nullptr != optarg) _options.ns = std::string(optarg) + "/"; break;

		default:
			usage("micrortps_agent");
			return -1;
		}
	}

	if (_options.poll_ms < 1) {
		_options.poll_ms = 1;
		printf("\033[1;33m[   micrortps_agent   ]\tPoll timeout too low, reset to 1 ms\033[0m");
	}

	if (_options.hw_flow_control && _options.sw_flow_control) {
		printf("\033[1;31m[   micrortps_agent   ]\tBoth HW and SW flow control set\033[0m");
		return -1;
	}

	return 0;
}

@[if recv_topics]@
/* Sender thread state */
std::atomic<bool> exit_sender_thread(false);
std::condition_variable t_send_queue_cv;
std::mutex t_send_queue_mutex;
std::queue<uint8_t> t_send_queue;

/* Sender thread routine. */
void t_send(void *)
{
	char data_buffer[BUFFER_SIZE] = {};
	size_t length = 0;
	uint8_t topic_ID = 255;

	while (running && !exit_sender_thread) {
		std::unique_lock<std::mutex> lk(t_send_queue_mutex);

		while (t_send_queue.empty() && !exit_sender_thread) {
			t_send_queue_cv.wait(lk);
		}

		topic_ID = t_send_queue.front();
		t_send_queue.pop();
		lk.unlock();

		size_t header_length = transport_node->get_header_length();
		/* make room for the header to fill in later */
		eprosima::fastcdr::FastBuffer cdrbuffer(&data_buffer[header_length], sizeof(data_buffer) - header_length);
		eprosima::fastcdr::Cdr scdr(cdrbuffer);

		if (!exit_sender_thread) {
			if (topics->getMsg(topic_ID, scdr)) {
				length = scdr.getSerializedDataLength();
        ssize_t write_res;

				if (0 < (write_res = transport_node->write(topic_ID, data_buffer, length))) {
					total_sent += static_cast<unsigned long int>(write_res);
					++sent;
				}
			}
		}
	}
}
@[end if]@

/* TODO Redo */
void signal_handler(int signum)
{
	printf("\n\033[1;33m[   micrortps_agent   ]\tGot signal (%d)\033[0m\n", signum);
	running = 0;
	transport_node->close();
}

int main(int argc, char ** argv)
{
  // Parse startup options
	if (parse_options(argc, argv)) {
    printf("\033[1;31m[   micrortps_agent   ]\tFailed to parse options\033[0m\n");
		exit(EXIT_FAILURE);
	}

  // TODO Use other module
	// register signal SIGINT and signal handler
	signal(SIGINT, signal_handler);

  // Initialize transport handlers
	printf("\033[0;37m--- MicroRTPS Agent ---\033[0m\n");
	printf("\033[1;33m[   micrortps_agent   ]\tStarting link...\033[0m\n");

	const char* localhost_only_var = std::getenv("ROS_LOCALHOST_ONLY");

	if ((localhost_only_var && strcmp(localhost_only_var, "1") == 0) || localhost_only) {
		printf("[   micrortps_agent   ]\tUsing only the local loopback network interface\n");
	}

	switch (_options.transport) {
	case options::eTransports::UART: {
			transport_node = std::make_unique<UART_node>(_options.device, _options.baudrate, _options.poll_ms,
						       _options.sw_flow_control, _options.hw_flow_control, _options.verbose_debug);
			printf("[   micrortps_agent   ]\tUART transport: device: %s; baudrate: %d; sleep time: %d us; poll interval: %d ms; flow_control: %s\n",
			       _options.device, _options.baudrate, _options.sleep_us, _options.poll_ms,
			       _options.sw_flow_control ? "SW enabled" : (_options.hw_flow_control ? "HW enabled" : "No"));
		}
		break;

	case options::eTransports::UDP: {
			transport_node = std::make_unique<UDP_node>(_options.ip, _options.recv_port, _options.send_port, _options.verbose_debug);
			printf("[   micrortps_agent   ]\tUDP transport: IP address: %s; inbound port: %u; outbound port: %u; sleep time: %d us\n",
			       _options.ip, _options.recv_port, _options.send_port, _options.sleep_us);
		}
		break;

	default:
		printf("\033[1;31m[   micrortps_agent   ]\tInvalid transport\033[0m\n");
		exit(EXIT_FAILURE);
	}

	if (0 > transport_node->init()) {
		printf("\033[1;31m[   micrortps_agent   ]\tFailed to initialize transport node\033[0m\n");
		exit(EXIT_FAILURE);
	}

  // Let threads initialize
	sleep(1);

@[if send_topics]@
  // Initialize data counters
	char data_buffer[BUFFER_SIZE] = {};
	unsigned long int received = 0, loop = 0;
	int length = 0;
  unsigned long int total_read = 0;
	bool receiving = false;
	uint8_t topic_ID = 255;
	std::chrono::time_point<std::chrono::steady_clock> start, end;
@[end if]@
  topics = std::make_unique<RtpsTopics>();

	// Initialize timesync module
	topics->set_timesync(std::make_shared<TimeSync>(_options.verbose_debug));

@[if recv_topics]@
  // Initialize RTPS topics handler
	topics->init(&t_send_queue_cv, &t_send_queue_mutex, &t_send_queue, _options.ns);
@[end if]@

	running = true;

@[if recv_topics]@
  // Start sender thread
	std::thread sender_thread(t_send, nullptr);
@[end if]@

  // Message processing routine
	while (running) {
@[if send_topics]@
		++loop;
		if (!receiving) { start = std::chrono::steady_clock::now(); }

		// Publish messages received from UART
		if (0 < (length = transport_node->read(&topic_ID, data_buffer, BUFFER_SIZE))) {
			topics->publish(topic_ID, data_buffer, sizeof(data_buffer));
			++received;
			total_read += static_cast<unsigned long int>(length);
			receiving = true;
			end = std::chrono::steady_clock::now();
		}
@[else]@
		usleep(_options.sleep_us);
@[end if]@
	}

@[if recv_topics]@
  // Kill and join sender thread
	exit_sender_thread = true;
	t_send_queue_cv.notify_one();
	sender_thread.join();
@[end if]@

@[if send_topics]@
	if (received > 0) {
	  std::chrono::duration<double> elapsed_secs = end - start;
		printf("\033[1;33m[   micrortps_agent   ]\tRECEIVED: %lu messages - %lu bytes; %lu loops - %.03f seconds - %.02f KB/s\033[0m\n",
		       received, total_read, loop, elapsed_secs.count(), static_cast<double>(total_read) / (1000 * elapsed_secs.count()));
	}
@[end if]@
@[if recv_topics]@
	if (sent > 0) {
		printf("\033[1;33m[   micrortps_agent   ]\tSENT: %lu messages - %lu bytes\033[0m\n",
           sent, total_sent);
	}
@[end if]@

  // Destroy transport handlers
	transport_node.reset();
  topics.reset();

	exit(EXIT_SUCCESS);
}
