#include <pdraw/pdraw_backend.h>

#include <pthread.h>
#include <stdio.h>
#include <string.h>

struct ctx {
	pthread_mutex_t mutex;
	pthread_cond_t cond;
	int status;
};

static void
pdraw_open_resp(struct pdraw_backend *pdraw, int status, void *userdata)
{
	struct ctx *ctx = userdata;
	printf("pdraw_open_resp: %d (%s)\n", status, strerror(-status));
	ctx->status = status;
	pthread_mutex_lock(&ctx->mutex);
	pthread_cond_signal(&ctx->cond);
	pthread_mutex_unlock(&ctx->mutex);
}

static void
pdraw_close_resp(struct pdraw_backend *pdraw, int status, void *userdata)
{
	struct ctx *ctx = userdata;
	printf("pdraw_close_resp: %d (%s)\n", status, strerror(-status));
	ctx->status = status;
	pthread_mutex_lock(&ctx->mutex);
	pthread_cond_signal(&ctx->cond);
	pthread_mutex_unlock(&ctx->mutex);
}

static const struct pdraw_backend_cbs backend_cbs = {
	.open_resp = pdraw_open_resp,
	.close_resp = pdraw_close_resp,
};

int main(int argc, char *argv[])
{
	struct pdraw_backend *pdraw;
	struct ctx ctx;
	int ret;

	if (argc < 2) {
		fprintf(stderr, "Missing argument\n");
		return 1;
	}

	pthread_mutex_init(&ctx.mutex, NULL);
	pthread_cond_init(&ctx.cond, NULL);

	ret = pdraw_be_new(&backend_cbs, &ctx, &pdraw);
	if (ret != 0) {
		fprintf(stderr, "pdraw_be_new: %d (%s)\n", ret, strerror(-ret));
		goto clean_cond;
	}

	ret = pdraw_be_open_url(pdraw, argv[1]);
	if (ret != 0) {
		fprintf(stderr,
			"pdraw_be_open_url: %d (%s)\n",
			ret,
			strerror(-ret));
		goto exit;
	}

	/* Wait for open resp */
	pthread_cond_wait(&ctx.cond, &ctx.mutex);
	if (ctx.status != 0)
		goto exit;

	ret = pdraw_be_close(pdraw);
	if (ret != 0) {
		fprintf(stderr,
			"pdraw_be_close: %d (%s)\n",
			ret,
			strerror(-ret));
		goto exit;
	}

	/* Wait for close resp */
	pthread_cond_wait(&ctx.cond, &ctx.mutex);

exit:
	pdraw_be_destroy(pdraw);
clean_cond:
	pthread_mutex_destroy(&ctx.mutex);
	pthread_cond_destroy(&ctx.cond);
	return ret;
}
