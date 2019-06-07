#include <pdraw/pdraw_backend.h>

#include <pthread.h>
#include <stdio.h>
#include <string.h>

struct ctx {
	pthread_mutex_t mutex;
	pthread_cond_t cond;
	int completed;
	int status;
};

static void ctx_signal_resp(struct ctx *ctx, int status)
{
	pthread_mutex_lock(&ctx->mutex);
	ctx->status = status;
	ctx->completed = 1;
	pthread_cond_signal(&ctx->cond);
	pthread_mutex_unlock(&ctx->mutex);
}

static int ctx_wait_resp(struct ctx *ctx)
{
	int status;

	pthread_mutex_lock(&ctx->mutex);
	while (!ctx->completed)
		pthread_cond_wait(&ctx->cond, &ctx->mutex);
	status = ctx->status;
	ctx->completed = 0;
	pthread_mutex_unlock(&ctx->mutex);

	return status;
}

static void
pdraw_stop_resp(struct pdraw_backend *pdraw, int status, void *userdata)
{
	struct ctx *ctx = userdata;

	printf("pdraw_stop_resp: %d (%s)\n", status, strerror(-status));

	ctx_signal_resp(ctx, status);
}

static void pdraw_demuxer_open_resp(struct pdraw_backend *pdraw,
				    struct pdraw_demuxer *demuxer,
				    int status,
				    void *userdata)
{
	struct ctx *ctx = userdata;

	printf("pdraw_demuxer_open_resp: %d (%s)\n", status, strerror(-status));

	ctx_signal_resp(ctx, status);
}

static void pdraw_demuxer_close_resp(struct pdraw_backend *pdraw,
				     struct pdraw_demuxer *demuxer,
				     int status,
				     void *userdata)
{
	struct ctx *ctx = userdata;

	printf("pdraw_demuxer_close_resp: %d (%s)\n",
	       status,
	       strerror(-status));

	ctx_signal_resp(ctx, status);
}

static const struct pdraw_backend_cbs backend_cbs = {
	.stop_resp = pdraw_stop_resp,
};

static const struct pdraw_backend_demuxer_cbs demuxer_cbs = {
	.open_resp = pdraw_demuxer_open_resp,
	.close_resp = pdraw_demuxer_close_resp,
};

int main(int argc, char *argv[])
{
	struct pdraw_backend *pdraw = NULL;
	struct pdraw_demuxer *demuxer = NULL;
	struct ctx ctx = {
		.mutex = PTHREAD_MUTEX_INITIALIZER,
		.cond = PTHREAD_COND_INITIALIZER,
	};
	int ret;

	if (argc < 2) {
		fprintf(stderr, "Missing argument\n");
		return 1;
	}

	ret = pdraw_be_new(&backend_cbs, &ctx, &pdraw);
	if (ret != 0) {
		fprintf(stderr, "pdraw_be_new: %d (%s)\n", ret, strerror(-ret));
		goto exit;
	}

	ret = pdraw_be_demuxer_new_from_url(
		pdraw, argv[1], &demuxer_cbs, &ctx, &demuxer);
	if (ret != 0) {
		fprintf(stderr,
			"pdraw_be_demuxer_new_from_url: %d (%s)\n",
			ret,
			strerror(-ret));
		goto exit;
	}

	/* Wait for demuxer open resp */
	if (ctx_wait_resp(&ctx) != 0)
		goto exit;

	/* The demuxer is now open. This is the point where your app should go
	 * in a waiting state while the video is running. The demuxer will call
	 * its `ready_to_play' callback when it is safe to call
	 * `pdraw_be_demuxer_play()' to start receiving frames */

	ret = pdraw_be_demuxer_close(pdraw, demuxer);
	if (ret != 0) {
		fprintf(stderr,
			"pdraw_be_demuxer_close: %d (%s)\n",
			ret,
			strerror(-ret));
		goto exit;
	}

	/* Wait for demuxer close resp */
	ctx_wait_resp(&ctx);

	/* Destroy the demuxer before stopping the backend */
	pdraw_be_demuxer_destroy(pdraw, demuxer);

	ret = pdraw_be_stop(pdraw);
	if (ret != 0) {
		fprintf(stderr,
			"pdraw_be_stop: %d (%s)\n",
			ret,
			strerror(-ret));
		goto exit;
	}

	/* Wait for stop resp */
	ctx_wait_resp(&ctx);

exit:
	if (pdraw != NULL)
		pdraw_be_destroy(pdraw);
	pthread_mutex_destroy(&ctx.mutex);
	pthread_cond_destroy(&ctx.cond);
	return ret;
}
