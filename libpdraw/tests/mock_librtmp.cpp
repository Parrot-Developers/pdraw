#include "mock_librtmp.hpp"
#include <errno.h>
#include <libpomp.h>
#include <stdlib.h>
#include <string.h>

struct librtmp_mock_control g_librtmp_mock = {};

struct rtmp_client {
	struct pomp_loop *loop;
	struct rtmp_callbacks cbs;
	void *userdata;
	bool connected;
	struct pomp_timer *conn_timer;
};

void librtmp_mock_reset(void)
{
	memset(&g_librtmp_mock, 0, sizeof(g_librtmp_mock));
}

struct rtmp_client *rtmp_client_new(struct pomp_loop *loop,
				    const struct rtmp_callbacks *cbs,
				    void *userdata)
{
	struct rtmp_client *client =
		(struct rtmp_client *)calloc(1, sizeof(*client));
	if (!client)
		return nullptr;
	client->loop = loop;
	client->cbs = *cbs;
	client->userdata = userdata;
	g_librtmp_mock.last_userdata = userdata;
	g_librtmp_mock.last_client = client;
	return client;
}

void rtmp_client_destroy(struct rtmp_client *client)
{
	if (client) {
		if (g_librtmp_mock.last_client == client) {
			g_librtmp_mock.last_client = nullptr;
		}
		if (client->conn_timer) {
			pomp_timer_destroy(client->conn_timer);
		}
		free(client);
	}
}

static void conn_timer_cb(struct pomp_timer *timer, void *userdata)
{
	auto *client = (struct rtmp_client *)userdata;
	if (g_librtmp_mock.trigger_connection_success_async) {
		/* Fire socket_cb with a dummy fd=-1 to exercise onSocketCreated
		 */
		if (client->cbs.socket_cb) {
			g_librtmp_mock.socket_cb_call_count++;
			client->cbs.socket_cb(-1, client->userdata);
		}
		client->connected = true;
		client->cbs.connection_state(
			RTMP_CLIENT_CONN_STATE_CONNECTED,
			RTMP_CLIENT_DISCONNECTION_REASON_UNKNOWN,
			client->userdata);
	} else if (g_librtmp_mock.trigger_disconnection_async) {
		client->connected = false;
		client->cbs.connection_state(
			RTMP_CLIENT_CONN_STATE_DISCONNECTED,
			g_librtmp_mock.disconnection_reason,
			client->userdata);
	}
}

int rtmp_client_connect(struct rtmp_client *client, const char *url)
{
	if (g_librtmp_mock.connect_should_fail) {
		return -g_librtmp_mock.connect_fail_errno;
	}

	if (!client->conn_timer) {
		client->conn_timer =
			pomp_timer_new(client->loop, conn_timer_cb, client);
	}
	pomp_timer_set(client->conn_timer, 1);

	client->cbs.connection_state(RTMP_CLIENT_CONN_STATE_CONNECTING,
				     RTMP_CLIENT_DISCONNECTION_REASON_UNKNOWN,
				     client->userdata);

	return 0;
}

int rtmp_client_disconnect(struct rtmp_client *client,
			   enum rtmp_client_disconnection_reason reason)
{
	client->connected = false;
	client->cbs.connection_state(
		RTMP_CLIENT_CONN_STATE_DISCONNECTED, reason, client->userdata);
	return 0;
}

int rtmp_client_flush(struct rtmp_client *client)
{
	g_librtmp_mock.flush_call_count++;
	return 0;
}

int rtmp_client_send_metadata(struct rtmp_client *client,
			      double duration,
			      int width,
			      int height,
			      double framerate,
			      int audio_sample_rate,
			      int audio_sample_size)
{
	g_librtmp_mock.send_metadata_call_count++;
	if (g_librtmp_mock.send_should_fail_eagain) {
		return -EAGAIN;
	}
	return 0;
}

int rtmp_client_send_packedmetadata(struct rtmp_client *client,
				    const uint8_t *buf,
				    size_t len,
				    uint32_t timestamp,
				    void *frame_userdata)
{
	return 0;
}

int rtmp_client_send_video_avcc(struct rtmp_client *client,
				const uint8_t *buf,
				size_t len,
				void *frame_userdata)
{
	g_librtmp_mock.send_video_avcc_call_count++;
	if (client->cbs.data_unref) {
		client->cbs.data_unref(
			(uint8_t *)buf, frame_userdata, client->userdata);
	}
	return 0;
}

int rtmp_client_send_video_frame(struct rtmp_client *client,
				 const uint8_t *buf,
				 size_t len,
				 uint32_t timestamp,
				 void *frame_userdata)
{
	g_librtmp_mock.send_video_frame_call_count++;
	if (g_librtmp_mock.send_should_fail_eagain) {
		return -EAGAIN;
	}
	if (client->cbs.data_unref) {
		client->cbs.data_unref(
			(uint8_t *)buf, frame_userdata, client->userdata);
	}
	return 0;
}

int rtmp_client_send_audio_specific_config(struct rtmp_client *client,
					   const uint8_t *buf,
					   size_t len,
					   void *frame_userdata)
{
	g_librtmp_mock.send_audio_specific_config_call_count++;
	if (client->cbs.data_unref) {
		client->cbs.data_unref(
			(uint8_t *)buf, frame_userdata, client->userdata);
	}
	return 0;
}

int rtmp_client_send_audio_data(struct rtmp_client *client,
				const uint8_t *buf,
				size_t len,
				uint32_t timestamp,
				void *frame_userdata)
{
	g_librtmp_mock.send_audio_data_call_count++;
	if (g_librtmp_mock.send_should_fail_eagain) {
		return -EAGAIN;
	}
	if (client->cbs.data_unref) {
		client->cbs.data_unref(
			(uint8_t *)buf, frame_userdata, client->userdata);
	}
	return 0;
}

int rtmp_client_set_socket_txbuf_size(struct rtmp_client *client, size_t size)
{
	g_librtmp_mock.last_txbuf_size = size;
	return 0;
}

int rtmp_anonymize_url(const char *url, char **anonymized)
{
	if (!url || !anonymized)
		return -EINVAL;
	*anonymized = strdup(url);
	return 0;
}

const char *rtmp_client_conn_state_str(enum rtmp_client_conn_state val)
{
	switch (val) {
	case RTMP_CLIENT_CONN_STATE_DISCONNECTED:
		return "DISCONNECTED";
	case RTMP_CLIENT_CONN_STATE_CONNECTING:
		return "CONNECTING";
	case RTMP_CLIENT_CONN_STATE_CONNECTED:
		return "CONNECTED";
	default:
		return "UNKNOWN";
	}
}

const char *
rtmp_client_disconnection_reason_str(enum rtmp_client_disconnection_reason val)
{
	switch (val) {
	case RTMP_CLIENT_DISCONNECTION_REASON_UNKNOWN:
		return "UNKNOWN";
	case RTMP_CLIENT_DISCONNECTION_REASON_CLIENT_REQUEST:
		return "CLIENT_REQUEST";
	case RTMP_CLIENT_DISCONNECTION_REASON_SERVER_REQUEST:
		return "SERVER_REQUEST";
	case RTMP_CLIENT_DISCONNECTION_REASON_NETWORK_ERROR:
		return "NETWORK_ERROR";
	case RTMP_CLIENT_DISCONNECTION_REASON_REFUSED:
		return "REFUSED";
	case RTMP_CLIENT_DISCONNECTION_REASON_ALREADY_IN_USE:
		return "ALREADY_IN_USE";
	case RTMP_CLIENT_DISCONNECTION_REASON_TIMEOUT:
		return "TIMEOUT";
	case RTMP_CLIENT_DISCONNECTION_REASON_INTERNAL_ERROR:
		return "INTERNAL_ERROR";
	default:
		return "UNKNOWN";
	}
}

void librtmp_mock_trigger_disconnection(
	enum rtmp_client_disconnection_reason reason)
{
	if (g_librtmp_mock.last_client) {
		g_librtmp_mock.last_client->connected = false;
		g_librtmp_mock.last_client->cbs.connection_state(
			RTMP_CLIENT_CONN_STATE_DISCONNECTED,
			reason,
			g_librtmp_mock.last_client->userdata);
	}
}

void librtmp_mock_trigger_peer_bw(uint32_t bandwidth)
{
	if (g_librtmp_mock.last_client &&
	    g_librtmp_mock.last_client->cbs.peer_bw_changed) {
		g_librtmp_mock.last_client->cbs.peer_bw_changed(
			bandwidth, g_librtmp_mock.last_client->userdata);
	}
}
