#include <stdbool.h>
#ifdef HOST_CONCURRENCY_STRESS
#include <pthread.h>
#include <stdint.h>
#define RUN_HOST_CONCURRENCY_STRESS
#endif
#include <stdio.h>
#include <string.h>

#include "../../../src/connection/sensor_data_snapshot.h"

static bool vector_is(const float *vector, size_t count, float expected)
{
	for (size_t i = 0; i < count; i++) {
		if (vector[i] != expected) {
			return false;
		}
	}
	return true;
}

#ifdef RUN_HOST_CONCURRENCY_STRESS
#define STRESS_ITERATIONS 500000U

struct stress_context {
	sensor_data_snapshot_t snapshot;
	bool producer_done;
	bool failed;
	uint32_t last_observed;
};

static void *stress_producer(void *arg)
{
	struct stress_context *ctx = arg;
	for (uint32_t value = 1; value <= STRESS_ITERATIONS; value++) {
		float q[4] = {value, value, value, value};
		float a[3] = {value, value, value};
		sensor_data_snapshot_publish_qa(&ctx->snapshot, q, a, (value & 1U) != 0, value);
	}
	__atomic_store_n(&ctx->producer_done, true, __ATOMIC_RELEASE);
	return NULL;
}

static void *stress_consumer(void *arg)
{
	struct stress_context *ctx = arg;
	for (;;) {
		if (sensor_data_snapshot_qa_pending(&ctx->snapshot)) {
			float q[4];
			float a[3];
			int64_t data_time;
			uint32_t generation;
			sensor_data_snapshot_read_qa(
				&ctx->snapshot, q, a, &data_time, &generation
			);
			uint32_t value = (uint32_t)q[0];
			if (!vector_is(q, 4, q[0]) || !vector_is(a, 3, q[0])
			    || data_time != value || generation != value
			    || value < ctx->last_observed) {
				__atomic_store_n(&ctx->failed, true, __ATOMIC_RELAXED);
				return NULL;
			}
			ctx->last_observed = value;
		}
		if (__atomic_load_n(&ctx->producer_done, __ATOMIC_ACQUIRE)
		    && !sensor_data_snapshot_qa_pending(&ctx->snapshot)) {
			return NULL;
		}
	}
}

static bool concurrent_stress_remained_coherent(void)
{
	struct stress_context ctx;
	memset(&ctx, 0, sizeof(ctx));
	pthread_t producer;
	pthread_t consumer;
	if (pthread_create(&producer, NULL, stress_producer, &ctx) != 0
	    || pthread_create(&consumer, NULL, stress_consumer, &ctx) != 0) {
		return false;
	}
	if (pthread_join(producer, NULL) != 0 || pthread_join(consumer, NULL) != 0) {
		return false;
	}
	return !__atomic_load_n(&ctx.failed, __ATOMIC_RELAXED)
		&& ctx.last_observed == STRESS_ITERATIONS;
}
#endif

int main(void)
{
	sensor_data_snapshot_t snapshot;
	memset(&snapshot, 0, sizeof(snapshot));
	float q[4] = {1, 1, 1, 1};
	float a[3] = {2, 2, 2};
	float m[3] = {3, 3, 3};
	float q_out[4];
	float a_out[3];
	float m_out[3];
	int64_t data_time;
	uint32_t generation;

	sensor_data_snapshot_publish_qa(&snapshot, q, a, true, 1234);
	sensor_data_snapshot_publish_m(&snapshot, m);
	if (!sensor_data_snapshot_qa_pending(&snapshot)
	    || !sensor_data_snapshot_m_pending(&snapshot)
	    || !sensor_data_snapshot_is_precise(&snapshot)) {
		fputs("FAIL: publication state was not visible\n", stderr);
		return 1;
	}
	sensor_data_snapshot_read_qa(
		&snapshot, q_out, a_out, &data_time, &generation
	);
	if (!vector_is(q_out, 4, 1) || !vector_is(a_out, 3, 2)
	    || data_time != 1234 || generation != 1
	    || sensor_data_snapshot_qa_pending(&snapshot)
	    || !sensor_data_snapshot_m_pending(&snapshot)) {
		fputs("FAIL: quaternion snapshot was incoherent or cleared the wrong state\n", stderr);
		return 1;
	}

	/* Timestamps can repeat at coarse kernel tick rates; generations cannot. */
	sensor_data_snapshot_publish_qa(&snapshot, q, a, false, 1234);
	sensor_data_snapshot_read_qm(
		&snapshot, q_out, m_out, &data_time, &generation
	);
	if (!vector_is(q_out, 4, 1) || !vector_is(m_out, 3, 3)
	    || data_time != 1234 || generation != 2
	    || sensor_data_snapshot_qa_pending(&snapshot)
	    || sensor_data_snapshot_m_pending(&snapshot)) {
		fputs("FAIL: combined snapshot was incoherent or remained pending\n", stderr);
		return 1;
	}
#ifdef RUN_HOST_CONCURRENCY_STRESS
	if (!concurrent_stress_remained_coherent()) {
		fputs("FAIL: concurrent producer/consumer observed a torn snapshot\n", stderr);
		return 1;
	}
#endif

	puts("PASS: snapshot remained coherent");
	return 0;
}
