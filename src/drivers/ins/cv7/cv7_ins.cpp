/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <lib/drivers/device/Device.hpp>

#include "cv7_ins.hpp"
#include "mip_sdk/src/mip/mip_all.h"
#include "mip_sdk/src/mip/mip_parser.h"
#include "CircularBuffer.hpp"

#define LOG_TRANSACTIONS

uint8_t external_heading_sensor_id = 1;
uint8_t gnss_antenna_sensor_id = 2;
uint8_t vehicle_frame_velocity_sensor_id = 3;

static CvIns *cv7_ins{nullptr};

// Temporary bugfix, CV7 is reporting SWU in relative position, not NED (as expected)
#define CONVERT_SWU_TO_NED

ModalIoSerial device_uart;

const uint8_t FILTER_ROLL_EVENT_ACTION_ID  = 1;
const uint8_t FILTER_PITCH_EVENT_ACTION_ID = 2;

void CvIns::cb_filter_llh(void *user, const mip_field *field, timestamp_type timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);
	mip_filter_position_llh_data data;
	if (extract_mip_filter_position_llh_data_from_field(field,&data)){
		ref->_f_llh.update_sample(data);
	}
}

void CvIns::cb_filter_atq(void *user, const mip_field *field, timestamp_type timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);
	mip_filter_attitude_quaternion_data data;

	if (extract_mip_filter_attitude_quaternion_data_from_field(field,&data)){
		ref->_f_quat.update_sample(data);
	}
}
void CvIns::cb_filter_ang_rate(void *user, const mip_field *field, timestamp_type timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);
	mip_filter_comp_angular_rate_data data;

	if (extract_mip_filter_comp_angular_rate_data_from_field(field,&data)){
		// PX4_INFO("%f, %f, %f", (double)data.gyro[0], (double)data.gyro[1], (double)data.gyro[2]);
		ref->_f_ang_rate.update_sample(data);
	}
}

void CvIns::cb_filter_rel_pos(void *user, const mip_field *field, timestamp_type timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);
	mip_filter_rel_pos_ned_data data;

	if (extract_mip_filter_rel_pos_ned_data_from_field(field,&data)){
		ref->_f_rel_pos.update_sample(data);
	}
}
void CvIns::cb_filter_vel_ned(void *user, const mip_field *field, timestamp_type timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);
	mip_filter_velocity_ned_data data;

	if (extract_mip_filter_velocity_ned_data_from_field(field,&data)){
		ref->_f_vel_ned.update_sample(data);
	}
}
void CvIns::cb_filter_lin_accel(void *user, const mip_field *field, timestamp_type timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);
	mip_filter_linear_accel_data data;

	if (extract_mip_filter_linear_accel_data_from_field(field,&data)){
		ref->_f_lin_veld.update_sample(data);
	}
}
void CvIns::cb_filter_status(void *user, const mip_field *field, timestamp_type timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);
	mip_filter_status_data data;

	if (extract_mip_filter_status_data_from_field(field,&data)){
		ref->_f_status.update_sample(data);
	}
}
void CvIns::cb_filter_meas_status(void *user, const mip_field *field, timestamp_type timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);
	mip_filter_aiding_measurement_summary_data data;

	if (extract_mip_filter_aiding_measurement_summary_data_from_field(field,&data)){
		ref->_f_aiding_summary.update_sample(data);
	}
}
void CvIns::cb_filter_pos_uncertainty(void *user, const mip_field *field, timestamp_type timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);
	mip_filter_position_llh_uncertainty_data data;

	if (extract_mip_filter_position_llh_uncertainty_data_from_field(field,&data)){
		ref->_f_pos_uncertainty.update_sample(data);
	}
}
void CvIns::cb_filter_vel_uncertainty(void *user, const mip_field *field, timestamp_type timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);
	mip_filter_velocity_ned_uncertainty_data data;

	if (extract_mip_filter_velocity_ned_uncertainty_data_from_field(field,&data)){
		ref->_f_vel_uncertainty.update_sample(data);
	}
}
void CvIns::cb_filter_atte_uncertainty(void *user, const mip_field *field, timestamp_type timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);
	mip_filter_euler_angles_uncertainty_data data;

	if (extract_mip_filter_euler_angles_uncertainty_data_from_field(field,&data)){
		ref->_f_atte_uncertainty.update_sample(data);
	}
}
void CvIns::cb_filter_comp_accel(void *user, const mip_field *field, timestamp_type timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);
	mip_filter_comp_accel_data data;

	if (extract_mip_filter_comp_accel_data_from_field(field,&data)){
		ref->_f_comp_accel.update_sample(data);
	}
}

void CvIns::cb_filter_timestamp(void *user, const mip_field *field, timestamp_type timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);
	// mip_filter_timestamp_data data;

	// if (extract_mip_filter_timestamp_data_from_field(field,&data))
	{
		// ref->_f_timestamp.update_sample(data);

		auto t = hrt_absolute_time() - (ref->_delay_offset * 1_us);
		float yaw = NAN;

		if (ref->_f_llh.updated) {
			vehicle_global_position_s gp{0};
			gp.timestamp = t;
			gp.timestamp_sample = t;
			gp.lat = ref->_f_llh.sample.latitude;
			gp.lon = ref->_f_llh.sample.longitude;
			gp.alt_ellipsoid = ref->_f_llh.sample.ellipsoid_height;
			gp.alt = ref->_f_llh.sample.ellipsoid_height;
			gp.eph = 0.1f;
			gp.epv = 0.1f;

			ref->_f_llh.updated = false;
			ref->_global_position_pub.publish(gp);
		}


		if (ref->_f_quat.updated) {
			vehicle_attitude_s att_data{0};
			att_data.timestamp = t;
			att_data.timestamp_sample = t;
			att_data.q[0] = ref->_f_quat.sample.q[0];
			att_data.q[1] = ref->_f_quat.sample.q[1];
			att_data.q[2] = ref->_f_quat.sample.q[2];
			att_data.q[3] = ref->_f_quat.sample.q[3];
			att_data.quat_reset_counter = 0;

			ref->_f_quat.updated = false;
			ref->_vehicle_attitude_pub.publish(att_data);

			// convert to YAW
			matrix::Eulerf euler_attitude(matrix::Quatf(att_data.q));
			yaw = math::degrees(euler_attitude.psi());
		}

		if (ref->_f_rel_pos.updated && ref->_f_vel_ned.updated) {
			vehicle_local_position_s vp{0};
			vp.timestamp = t;
			vp.timestamp_sample = t;
			vp.x = ref->_f_rel_pos.sample.relative_position[0];
			vp.y = ref->_f_rel_pos.sample.relative_position[1];
			vp.z = ref->_f_rel_pos.sample.relative_position[2];
			#ifdef CONVERT_SWU_TO_NED
			vp.x = vp.x * -1.f;
			vp.y = vp.y * -1.f;
			vp.z = vp.z * -1.f;
			#endif

			vp.vx = ref->_f_vel_ned.sample.north;
			vp.vy = ref->_f_vel_ned.sample.east;
			vp.vz = ref->_f_vel_ned.sample.down;

			vp.ax = ref->_f_comp_accel.sample.accel[0];
			vp.ay = ref->_f_comp_accel.sample.accel[1];
			vp.az = ref->_f_comp_accel.sample.accel[2];

			if(ref->_f_pos_uncertainty.updated){
				vp.eph = sqrt(ref->_f_pos_uncertainty.sample.north*ref->_f_pos_uncertainty.sample.north + ref->_f_pos_uncertainty.sample.east*ref->_f_pos_uncertainty.sample.east);
				vp.epv = ref->_f_pos_uncertainty.sample.down;
			}
			if(ref->_f_vel_uncertainty.updated){
				vp.evh = sqrt(ref->_f_vel_uncertainty.sample.north*ref->_f_vel_uncertainty.sample.north + ref->_f_vel_uncertainty.sample.east*ref->_f_vel_uncertainty.sample.east);
				vp.evv = ref->_f_pos_uncertainty.sample.down;
			}
			if(ref->_f_atte_uncertainty.updated){
				vehicle_odometry_s vo{0};
				vo.timestamp = t;
				vo.timestamp_sample = t;

				vo.position[0] = ref->_f_rel_pos.sample.relative_position[0];
				vo.position[1] = ref->_f_rel_pos.sample.relative_position[1];
				vo.position[2] = ref->_f_rel_pos.sample.relative_position[2];
				#ifdef CONVERT_SWU_TO_NED
				for (size_t i = 0; i < 3; i++)
				{
					vo.position[i] = vo.position[i] * -1.f;
				}
				#endif

				vo.velocity[0] = ref->_f_vel_ned.sample.north;
				vo.velocity[1] = ref->_f_vel_ned.sample.east;
				vo.velocity[2] = ref->_f_vel_ned.sample.down;

				vo.position_variance[0] = ref->_f_pos_uncertainty.sample.north;
				vo.position_variance[1] = ref->_f_pos_uncertainty.sample.east;
				vo.position_variance[2] = ref->_f_pos_uncertainty.sample.down;

				vo.velocity_variance[0] = ref->_f_vel_uncertainty.sample.north;
				vo.velocity_variance[1] = ref->_f_vel_uncertainty.sample.east;
				vo.velocity_variance[2] = ref->_f_vel_uncertainty.sample.down;

				vo.orientation_variance[0] = ref->_f_atte_uncertainty.sample.roll;
				vo.orientation_variance[1] = ref->_f_atte_uncertainty.sample.pitch;
				vo.orientation_variance[2] = ref->_f_atte_uncertainty.sample.yaw;

				ref->_vehicle_odometry_pub.publish(vo);
			}

			vp.delta_xy[0] = 0.f;
			vp.delta_xy[1] = 0.f;
			vp.xy_reset_counter = 0;
			vp.z = ref->_f_rel_pos.sample.relative_position[2];

			vp.z_valid = true;
			vp.xy_valid = true;
			vp.v_xy_valid = true;
			vp.v_z_valid = true;
			vp.z_deriv = vp.vz;
			vp.z_global = true;
			vp.xy_global = true;
			// Set heading as valid if we have any yaw
			vp.heading_good_for_control = (yaw != NAN);
			vp.heading = yaw; // This is extracted from vehicle attitude

			vp.vxy_max = INFINITY;
			vp.vz_max = INFINITY;
			vp.hagl_min = INFINITY;
			vp.hagl_max = INFINITY;

			ref->_f_rel_pos.updated = false;
			ref->_f_vel_ned.updated = false;
			ref->_vehicle_local_position_pub.publish(vp);
		}

		if (ref->_f_ang_rate.updated) {
			vehicle_angular_velocity_s av{0};
			av.timestamp = t;
			av.timestamp_sample = t;

			av.xyz[0] = ref->_f_ang_rate.sample.gyro[0];
			av.xyz[1] = ref->_f_ang_rate.sample.gyro[1];
			av.xyz[2] = ref->_f_ang_rate.sample.gyro[2];

			// xyz_derivative ??
			ref->_f_ang_rate.updated = false;
			ref->_vehicle_angular_velocity_pub.publish(av);
		}

		if(ref->_f_status.updated || ref->_f_aiding_summary.updated){
			debug_array_s dbg{0};
			dbg.id = 0x01;
			dbg.timestamp = t;
			strcpy(dbg.name,"CV7");
			dbg.data[0] = ref->_f_status.sample.filter_state * 1.0f;
			dbg.data[1] = ref->_f_status.sample.dynamics_mode * 1.0f;
			dbg.data[2] = ref->_f_status.sample.status_flags * 1.0f;
			dbg.data[3] = ref->_f_aiding_summary.sample.indicator * 1.0f;
			dbg.data[4] = ref->_f_aiding_summary.sample.time_of_week * 1.0f;
			dbg.data[5] = ref->_f_aiding_summary.sample.source * 1.0f;
			dbg.data[6] = ref->_f_aiding_summary.sample.type * 1.0f;

			ref->_debug_array_pub.publish(dbg);

			ref->_f_status.updated = false;
			ref->_f_aiding_summary.updated = false;
		}

#if 1
		// Estimator Status
		// TODO: for now we only fullfill components needed by the commander
		estimator_status_s status{0};
		status.timestamp = t;
		status.timestamp_sample = t;
		// Note: These flags require insight into the inner operations of the filter.
		//       Below is a minimal mapping of error flags from CV7 to the PX4 health flags
		// Filter State Mapping, if the filter is in 4, set merging GPS and Height being fused flags
		// Only the gps flag is inspected in the checks
		status.control_mode_flags = ref->_f_status.sample.filter_state == 4 ? (0x1 << estimator_status_s::CS_GPS) | (0x1 << estimator_status_s::CS_GPS_HGT) : 0x00;
		// If there is a general filter condition, set a fault flag to trigger an issue
		status.filter_fault_flags = (ref->_f_status.sample.status_flags & MIP_FILTER_STATUS_FLAGS_GQ7_FILTER_CONDITION) ? 0x1 : 0x00;
		status.innovation_check_flags = 0;
		status.gps_check_fail_flags = 0;

		status.mag_test_ratio = 0.1f;
		status.vel_test_ratio = 0.1f;
		status.pos_test_ratio = 0.1f;
		status.hgt_test_ratio = 0.1f;
		status.tas_test_ratio = 0.1f;
		status.hagl_test_ratio = 0.1f;
		status.beta_test_ratio = 0.1f;

		status.pos_horiz_accuracy = sqrt(ref->_f_pos_uncertainty.sample.north*ref->_f_pos_uncertainty.sample.north + ref->_f_pos_uncertainty.sample.east*ref->_f_pos_uncertainty.sample.east);
		status.pos_vert_accuracy = ref->_f_pos_uncertainty.sample.down;
		status.solution_status_flags = 0;

		status.time_slip = 0;
		status.pre_flt_fail_innov_heading = false;
		status.pre_flt_fail_innov_vel_horiz = false;
		status.pre_flt_fail_innov_vel_vert = false;
		status.pre_flt_fail_innov_height = false;
		status.pre_flt_fail_mag_field_disturbed = false;
		status.accel_device_id = ref->_config.device_id;
		status.gyro_device_id = ref->_config.device_id;
		status.mag_device_id = ref->_config.device_id;
		status.baro_device_id = ref->_config.device_id;
		ref->_estimator_status_pub.publish(status);

		sensor_selection_s sensor_selection{};
		sensor_selection.accel_device_id = ref->_config.device_id;
		sensor_selection.gyro_device_id = ref->_config.device_id;
		sensor_selection.timestamp = t;
		ref->_sensor_selection_pub.publish(sensor_selection);
#endif

	}
}

void CvIns::cb_accel(void *user, const mip_field *field, timestamp_type timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);
	mip_sensor_scaled_accel_data data;

	if (extract_mip_sensor_scaled_accel_data_from_field(field, &data)) {
		ref->_accel.update_sample(data);
	}
}

void CvIns::cb_gyro(void *user, const mip_field *field, timestamp_type timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);
	mip_sensor_scaled_gyro_data data;

	if (extract_mip_sensor_scaled_gyro_data_from_field(field, &data)) {
		ref->_gyro.update_sample(data);
	}
}

void CvIns::cb_mag(void *user, const mip_field *field, timestamp_type timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);
	mip_sensor_scaled_mag_data data;

	if (extract_mip_sensor_scaled_mag_data_from_field(field, &data)) {
		ref->_mag.update_sample(data);
	}
}

void CvIns::cb_baro(void *user, const mip_field *field, timestamp_type timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);
	mip_sensor_scaled_pressure_data data;

	PX4_DEBUG("[BARO] Now %" PRIu64 " Then %" PRIu64 " Elapsed %" PRIu64, hrt_absolute_time(), timestamp,
		  hrt_elapsed_time(&timestamp));

	if (extract_mip_sensor_scaled_pressure_data_from_field(field, &data)) {
		ref->_baro.update_sample(data);
	}
}

void CvIns::cb_ref_timestamp(void *user, const mip_field *field, timestamp_type timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);
	mip_shared_reference_timestamp_data data;

	if (extract_mip_shared_reference_timestamp_data_from_field(field, &data)) {

		// Convert to a useful time for PX4
		// auto t = timestamp - 1900_us;				// Packets are then ~4ms old and timestamp duplications
		// auto t = hrt_absolute_time() - 1900_us;				// Packets are then ~2ms old
		// auto t = hrt_absolute_time() - (_param_cv7_delay.get() ? 1900_us : 0_us);				// Packets are then ~2ms old
		auto t = hrt_absolute_time() - (ref->_delay_offset * 1_us);				// Packets are then ~2ms old
		// auto t = hrt_absolute_time();				// Packets are old but system thinks they are new
		// auto t = timestamp;						// Packets at time of arrival and timestamp duplications

		// Send all of the data with the common timestamp

		if (ref->_accel.updated) {
			ref->_px4_accel.update(t, ref->_accel.sample.scaled_accel[0]*CONSTANTS_ONE_G,
					       ref->_accel.sample.scaled_accel[1]*CONSTANTS_ONE_G,
					       ref->_accel.sample.scaled_accel[2]*CONSTANTS_ONE_G);
			ref->update_imu_sample_time(t);
			ref->_accel.updated = false;
		}

		if (ref->_gyro.updated) {
			ref->_px4_gyro.update(t, ref->_gyro.sample.scaled_gyro[0], ref->_gyro.sample.scaled_gyro[1],
					      ref->_gyro.sample.scaled_gyro[2]);
			ref->update_imu_sample_time(t);
			ref->_gyro.updated = false;
		}

		if (ref->_mag.updated) {
			ref->_px4_mag.update(t, ref->_mag.sample.scaled_mag[0], ref->_mag.sample.scaled_mag[1], ref->_mag.sample.scaled_mag[2]);
			ref->_mag.updated = false;
		}

		if (ref->_baro.updated) {
			ref->_sensor_baro.timestamp = timestamp;
			ref->_sensor_baro.timestamp_sample = t;
			ref->_sensor_baro.pressure = ref->_baro.sample.scaled_pressure * 100.f; // convert [Pa] to [mBar]
			ref->_sensor_baro_pub.publish(ref->_sensor_baro);
			ref->_baro.updated = false;
		}

		// Now process the filter information
		cb_filter_timestamp(user,field,timestamp);
	}
}



void handle_filter_event_source(void *user, const mip_field *field, timestamp_type timestamp)
{
	mip_shared_event_source_data data;

	if (extract_mip_shared_event_source_data_from_field(field, &data)) {
		if (data.trigger_id == FILTER_ROLL_EVENT_ACTION_ID) {
			PX4_WARN("WARNING: Roll event triggered!\n");

		} else if (data.trigger_id == FILTER_PITCH_EVENT_ACTION_ID) {
			PX4_WARN("WARNING: Pitch event triggered!\n");
		}
	}
}

timestamp_type get_current_timestamp()
{
	return hrt_absolute_time();
}

bool mip_interface_user_recv_from_device(mip_interface *device, uint8_t *buffer, size_t max_length, timeout_type wait_time,
		size_t *out_length, timestamp_type *timestamp_out)
{
	(void)device;

	*timestamp_out = hrt_absolute_time();

	int res = device_uart.uart_read(buffer, max_length);

	if (res == -1 && errno != EAGAIN) {
		PX4_DEBUG("RX 1 %d(%d)",res,max_length);
		*out_length = 0;
		return false;
	}

	if (res >= 0) {
		*out_length = res;
		PX4_DEBUG("RX 2 %d(%d)",*out_length,max_length);
	}

	PX4_DEBUG("RX 3 %d(%d)",*out_length,max_length);

#ifdef LOG_TRANSACTIONS

	if (cv7_ins) {
		cv7_ins->get_logger().enqueue_rx(buffer, *out_length);
	}

#endif
	cv7_ins->_debug_rx_bytes[0] = math::min<uint32_t>(cv7_ins->_debug_rx_bytes[0], *out_length);
	cv7_ins->_debug_rx_bytes[1] += *out_length;
	cv7_ins->_debug_rx_bytes[2] = math::max<uint32_t>(cv7_ins->_debug_rx_bytes[2], *out_length);
	cv7_ins->_debug_rx_bytes[3]++;
	return true;
}

bool mip_interface_user_send_to_device(mip_interface *device, const uint8_t *data, size_t length)
{


#ifdef LOG_TRANSACTIONS

	if (cv7_ins) {
		cv7_ins->get_logger().enqueue_tx(data, length);
	}

#endif

	PX4_DEBUG("TX %d", length);
	int res = device_uart.uart_write(const_cast<uint8_t *>(data), length);

	if(cv7_ins){
		cv7_ins->_debug_tx_bytes += length;
	}

	if (res >= 0) {
		return true;
	}

	return false;

}

CvIns::CvIns(const char *uart_port, int32_t rot) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
	/* store port name */
	memset(_port,'\0', 20);
	int max_len = math::min((unsigned int)20,strlen(uart_port));
	strncpy(_port, uart_port, max_len);
	/* enforce null termination */
	_port[19] = '\0';
	_config.rot = static_cast<Rotation>(rot);

	// // Clamp rate to allowable ranges
	_config.sens_imu_update_rate_hz = math::constrain<uint16_t>(_param_cv7_update_rate.get(),100,1000);

	device::Device::DeviceId device_id{};
	device_id.devid_s.devtype = DRV_INS_DEVTYPE_3DMCV7;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;
	device_id.devid_s.bus = 2;
	_config.device_id = device_id.devid;
	// Default to ROTATION_NONE
	_px4_accel.set_device_id(_config.device_id);
	_px4_gyro.set_device_id(_config.device_id);
	_px4_mag.set_device_id(_config.device_id);

	// Set the default values for the baro (which may not change)
	_sensor_baro.device_id = _config.device_id;
	_sensor_baro.pressure = 0;
	_sensor_baro.temperature = 0;
	_sensor_baro.error_count = 0;
}

CvIns::~CvIns()
{
	if (device_uart.is_open()) {
		device_uart.uart_close();
	}

	_sensor_baro_pub.unadvertise();
	//delete &_sensor_baro_pub;

#ifdef LOG_TRANSACTIONS
	_logger.thread_stop();
#endif
	PX4_INFO("Destructor");
	_sensor_baro_pub.unadvertise();
	_sensor_selection_pub.unadvertise();
	_vehicle_local_position_pub.unadvertise();
	_vehicle_angular_velocity_pub.unadvertise();
	_vehicle_attitude_pub.unadvertise();
	_global_position_pub.unadvertise();
	_vehicle_odometry_pub.unadvertise();
	_debug_array_pub.unadvertise();
	_estimator_status_pub.unadvertise();


	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool CvIns::init()
{
	// Run on fixed interval
	ScheduleOnInterval(_param_cv7_schedule.get());

	return true;
}

void CvIns::set_sensor_rate(mip_descriptor_rate *sensor_descriptors, uint16_t len)
{
	// Get the base rate
	uint16_t sensor_base_rate;

	if (mip_3dm_get_base_rate(&device, MIP_SENSOR_DATA_DESC_SET, &sensor_base_rate) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not get sensor base rate format!");
		return;
	}

	PX4_INFO("The CV7 base rate is %d", sensor_base_rate);

	for (uint16_t i = 0; i < len; i++) {
		// Compute the desired decimation and update all of the sensors in this set
		float sensor_decimation = static_cast<float>(sensor_base_rate) / static_cast<float>(sensor_descriptors[i].decimation);

		sensor_descriptors[i].decimation = static_cast<uint16_t>(sensor_decimation);
	}

	// Write the settings
	mip_cmd_result res = mip_3dm_write_message_format(&device, MIP_SENSOR_DATA_DESC_SET, len, sensor_descriptors);

	if (res != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not set sensor message format! Result of %d", res);
		return;
	}
}

int CvIns::connect_at_baud(int32_t baud)
{
	if (device_uart.is_open()) {
		if (device_uart.uart_set_baud(baud) == PX4_ERROR) {
			PX4_INFO(" - Failed to set UART %" PRIu32 " baud", baud);
		}

	} else if (device_uart.uart_open(_port, baud) == PX4_ERROR) {
		PX4_INFO(" - Failed to open UART");
		PX4_ERR("ERROR: Could not open device port!");
		return PX4_ERROR;
	}

	PX4_INFO("Serial Port %s with baud of %" PRIu32 " baud", (device_uart.is_open() ? "CONNECTED" : "NOT CONNECTED"), baud);

	// Re-init the interface with the correct timeouts
	// mip_interface_init(&device, parse_buffer, sizeof(parse_buffer), mip_timeout_from_baudrate(baud) * 1_ms, 250_ms);
	mip_interface_init(&device, parse_buffer, sizeof(parse_buffer), mip_timeout_from_baudrate(baud) * 1_ms, 250_ms,
			&mip_interface_user_send_to_device, &mip_interface_user_recv_from_device, &mip_interface_default_update, NULL);

	PX4_INFO("mip_base_ping");

	if (mip_base_ping(&device) != MIP_ACK_OK) {
		PX4_INFO(" - Failed to Ping 1");
		usleep(200_ms);

		if (mip_base_ping(&device) != MIP_ACK_OK) {
			PX4_INFO(" - Failed to Ping 2");
			return PX4_ERROR;
		}
	}

	PX4_INFO("Successfully opened and pinged");
	return PX4_OK;
}

void CvIns::initialize_cv7()
{
	if (_is_initialized) {
		return;
	}

	// first try default baudrate
	const uint32_t DEFAULT_BAUDRATE = 115200;
	const uint32_t DESIRED_BAUDRATE = 921600;

	if (connect_at_baud(DEFAULT_BAUDRATE) == PX4_ERROR) {

		static constexpr uint32_t BAUDRATES[] {9600, 19200, 38400, 57600, 115200, 128000, 230400, 460800, 921600};
		bool is_connected = false;

		for (auto &baudrate : BAUDRATES) {
			if (connect_at_baud(baudrate) == PX4_OK) {
				PX4_INFO("found baudrate %" PRIu32, baudrate);
				is_connected = true;
				break;
			}
		}

		if (!is_connected) {
			_is_init_failed = true;
			PX4_WARN("Could not connect to the device, exiting");
			return;
		}
	}


	PX4_INFO("mip_base_set_idle");

	if (mip_base_set_idle(&device) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not set the device to idle!");
		return;
	}

// Disable the reset of the device
#if 0
	PX4_INFO("Setting to default device settings");

	//Load the device default settings (so the device is in a known state)
	if (mip_3dm_default_device_settings(&device) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not load default device settings!");
		return;
	}

	PX4_INFO("Connecting at default baudrate");

	if (connect_at_baud(DEFAULT_BAUDRATE) == PX4_ERROR) {
		PX4_ERR("ERROR: Could not reconnect at expected baud!");
		return;
	}
#endif
	PX4_INFO("Setting the baud to desired baud rate");

	usleep(500_ms);

	if (mip_3dm_write_uart_baudrate(&device, DESIRED_BAUDRATE) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not set the baudrate!");
		_is_init_failed = true;
		return;
	}

	tcflush(device_uart.uart_get_fd(), TCIOFLUSH);

	usleep(500_ms);

	for (int i = 0; i < 10; i++) {
		PX4_INFO("Connection Attempt: %d", i);

		if (connect_at_baud(DESIRED_BAUDRATE) == PX4_OK) {
			break;
		}

		if (i >= 9) {
			PX4_ERR("ERROR: Could not reconnect at desired baud!");
			_is_init_failed = true;
			return;
		}
	}

	switch (_config.selected_mode) {
	case mode_imu: {
			// Scaled Gyro and Accel at a high rate
			mip_descriptor_rate imu_sensors[5] = {
				{ MIP_DATA_DESC_SENSOR_ACCEL_SCALED, _config.sens_imu_update_rate_hz},
				{ MIP_DATA_DESC_SENSOR_GYRO_SCALED, _config.sens_imu_update_rate_hz},
				{ MIP_DATA_DESC_SENSOR_MAG_SCALED,  _config.sens_other_update_rate_hz},
				{ MIP_DATA_DESC_SENSOR_PRESSURE_SCALED, _config.sens_other_update_rate_hz},
				{ MIP_DATA_DESC_SHARED_REFERENCE_TIME, _config.sens_imu_update_rate_hz},

			};

			set_sensor_rate(imu_sensors, 5);

			//
			// Register data callbacks
			//
			mip_interface_register_field_callback(&device, &sensor_data_handlers[0], MIP_SENSOR_DATA_DESC_SET,
							      MIP_DATA_DESC_SENSOR_ACCEL_SCALED, &cb_accel, this);
			mip_interface_register_field_callback(&device, &sensor_data_handlers[1], MIP_SENSOR_DATA_DESC_SET,
							      MIP_DATA_DESC_SENSOR_GYRO_SCALED, &cb_gyro, this);
			mip_interface_register_field_callback(&device, &sensor_data_handlers[2], MIP_SENSOR_DATA_DESC_SET,
							      MIP_DATA_DESC_SENSOR_MAG_SCALED, &cb_mag, this);
			mip_interface_register_field_callback(&device, &sensor_data_handlers[3], MIP_SENSOR_DATA_DESC_SET,
							      MIP_DATA_DESC_SENSOR_PRESSURE_SCALED, &cb_baro, this);
			mip_interface_register_field_callback(&device, &sensor_data_handlers[4], MIP_SHARED_DATA_DESC_SET,
							      MIP_DATA_DESC_SHARED_REFERENCE_TIME, &cb_ref_timestamp, this);

	// 	}
	// 	break;

	// case mode_ahrs: {
	// 	}
	// 	break;

	// case mode_ins: {

		//
		//External GNSS antenna reference frame
		//
		mip_aiding_frame_config_command_rotation rotation{0};
		for (uint8_t i = 0; i < 3; i++)
		{
			rotation.euler[i] = 0.0;
		}
		float translation[3] = { (float)_param_cv7_gps_x.get(), (float)_param_cv7_gps_y.get(), (float)_param_cv7_gps_z.get() };
		mip_aiding_write_frame_config(&device,gnss_antenna_sensor_id,MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_EULER,false,translation,&rotation);


		// Get the base rate
		uint16_t filter_base_rate;

		if (mip_3dm_get_base_rate(&device, MIP_FILTER_DATA_DESC_SET, &filter_base_rate) != MIP_ACK_OK) {
			PX4_ERR("ERROR: Could not get sensor base rate format!");
			return;
		}

		PX4_INFO("The CV7 base rate is %d", filter_base_rate);

		// const uint16_t filter_sample_rate = _config.sens_imu_update_rate_hz; // Hz
		// const uint16_t filter_decimation = filter_base_rate / filter_sample_rate;

		// Scaled Gyro and Accel at a high rate
		mip_descriptor_rate filter_data[13] = {
			{ MIP_DATA_DESC_FILTER_POS_LLH, _config.sens_imu_update_rate_hz},
			{ MIP_DATA_DESC_FILTER_ATT_QUATERNION,  _config.sens_imu_update_rate_hz},
			{ MIP_DATA_DESC_FILTER_COMPENSATED_ANGULAR_RATE, _config.sens_imu_update_rate_hz},
			{ MIP_DATA_DESC_FILTER_REL_POS_NED, _config.sens_imu_update_rate_hz},
			{ MIP_DATA_DESC_FILTER_VEL_NED, _config.sens_imu_update_rate_hz},
			{ MIP_DATA_DESC_FILTER_LINEAR_ACCELERATION, _config.sens_other_update_rate_hz},
			{ MIP_DATA_DESC_FILTER_FILTER_STATUS, _config.sens_status_update_rate_hz},
			{ MIP_DATA_DESC_FILTER_POS_UNCERTAINTY, _config.sens_imu_update_rate_hz},
			{ MIP_DATA_DESC_FILTER_VEL_UNCERTAINTY, _config.sens_imu_update_rate_hz},
			{ MIP_DATA_DESC_FILTER_ATT_UNCERTAINTY_EULER, _config.sens_imu_update_rate_hz},
			{ MIP_DATA_DESC_SHARED_REFERENCE_TIME, _config.sens_imu_update_rate_hz},
			{ MIP_DATA_DESC_FILTER_AID_MEAS_SUMMARY, _config.sens_status_update_rate_hz},
			{ MIP_DATA_DESC_FILTER_COMPENSATED_ACCELERATION, _config.sens_status_update_rate_hz},
		};

		for (uint16_t i = 0; i < 13; i++) {
			// Compute the desired decimation and update all of the sensors in this set
			float sensor_decimation = static_cast<float>(filter_base_rate) / static_cast<float>(filter_data[i].decimation);

			filter_data[i].decimation = static_cast<uint16_t>(sensor_decimation);
		}

		// Write the settings
		mip_cmd_result res = mip_3dm_write_message_format(&device, MIP_FILTER_DATA_DESC_SET, 13, filter_data);

		if (res != MIP_ACK_OK) {
			PX4_ERR("ERROR: Could not set filter message format! Result of %d", res);
			return;
		}

		mip_interface_register_field_callback(&device, &filter_data_handlers[0], MIP_FILTER_DATA_DESC_SET,
							MIP_DATA_DESC_FILTER_POS_LLH, &cb_filter_llh, this);
		mip_interface_register_field_callback(&device, &filter_data_handlers[1], MIP_FILTER_DATA_DESC_SET,
							MIP_DATA_DESC_FILTER_ATT_QUATERNION, &cb_filter_atq, this);
		mip_interface_register_field_callback(&device, &filter_data_handlers[2], MIP_FILTER_DATA_DESC_SET,
							MIP_DATA_DESC_FILTER_COMPENSATED_ANGULAR_RATE, &cb_filter_ang_rate, this);
		mip_interface_register_field_callback(&device, &filter_data_handlers[3], MIP_FILTER_DATA_DESC_SET,
							MIP_DATA_DESC_FILTER_REL_POS_NED, &cb_filter_rel_pos, this);
		mip_interface_register_field_callback(&device, &filter_data_handlers[4], MIP_FILTER_DATA_DESC_SET,
							MIP_DATA_DESC_FILTER_VEL_NED, &cb_filter_vel_ned, this);
		mip_interface_register_field_callback(&device, &filter_data_handlers[5], MIP_FILTER_DATA_DESC_SET,
							MIP_DATA_DESC_FILTER_LINEAR_ACCELERATION, &cb_filter_lin_accel, this);
		mip_interface_register_field_callback(&device, &filter_data_handlers[6], MIP_FILTER_DATA_DESC_SET,
							MIP_DATA_DESC_FILTER_FILTER_STATUS, &cb_filter_status, this);
		mip_interface_register_field_callback(&device, &filter_data_handlers[7], MIP_FILTER_DATA_DESC_SET,
							MIP_DATA_DESC_FILTER_POS_UNCERTAINTY, &cb_filter_pos_uncertainty, this);
		mip_interface_register_field_callback(&device, &filter_data_handlers[8], MIP_SHARED_DATA_DESC_SET,
							MIP_DATA_DESC_SHARED_REFERENCE_TIME, &cb_ref_timestamp, this);
		mip_interface_register_field_callback(&device, &filter_data_handlers[9], MIP_FILTER_DATA_DESC_SET,
							MIP_DATA_DESC_FILTER_AID_MEAS_SUMMARY, &cb_filter_meas_status, this);
		mip_interface_register_field_callback(&device, &filter_data_handlers[10], MIP_FILTER_DATA_DESC_SET,
							MIP_DATA_DESC_FILTER_VEL_UNCERTAINTY, &cb_filter_vel_uncertainty, this);
		mip_interface_register_field_callback(&device, &filter_data_handlers[11], MIP_FILTER_DATA_DESC_SET,
							MIP_DATA_DESC_FILTER_ATT_UNCERTAINTY_EULER, &cb_filter_atte_uncertainty, this);
		mip_interface_register_field_callback(&device, &filter_data_handlers[12], MIP_FILTER_DATA_DESC_SET,
							MIP_DATA_DESC_FILTER_COMPENSATED_ACCELERATION, &cb_filter_atte_uncertainty, this);

		// Selectively turn on the mag aiding source
		if(_param_cv7_int_mag_en.get() == 1)
		{
			if(mip_filter_write_aiding_measurement_enable(&device, MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_MAGNETOMETER, true) != MIP_ACK_OK)
				PX4_ERR("ERROR: Could not set filter aiding measurement enable!");
		}

		if(mip_filter_write_aiding_measurement_enable(&device, MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_GNSS_POS_VEL, true) != MIP_ACK_OK)
			PX4_ERR("ERROR: Could not set filter aiding measurement enable!");


		#ifdef LOG_TRANSACTIONS
		// Only when logging the MIP data, enable factory streaming support
		// This will echo commands to the device and provide necessary information
		// for replay using the factory utilities
		if(mip_3dm_factory_streaming(&device,MIP_3DM_FACTORY_STREAMING_COMMAND_ACTION_MERGE,0x00)!= MIP_ACK_OK)
			PX4_ERR("ERROR: Could not enable factory streaming");
		#endif

		// Reset the filter, then set the initial conditions
		if(mip_filter_reset(&device) != MIP_ACK_OK)
			PX4_ERR("ERROR: Could not reset the filter!");


		float filter_init_pos[3] = {0};
		float filter_init_vel[3] = {0};
		uint8_t initial_alignment = MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_MAGNETOMETER;
		if(_param_cv7_alignment.get() == 1){
			initial_alignment = MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_KINEMATIC;
		}
		if(_param_cv7_alignment.get() == 2){
			initial_alignment = MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_EXTERNAL;
		}
		if(_param_cv7_alignment.get() == 3){
			initial_alignment = MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_DUAL_ANTENNA;
		}
		// Config for Position, Velocity and Attitude, use kinematic alignment for initialization
		if(mip_filter_write_initialization_configuration(&device, 0, MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_INITIAL_CONDITION_SOURCE_AUTO_POS_VEL_ATT,
		initial_alignment,
		0.0, 0.0, 0.0, filter_init_pos, filter_init_vel, MIP_FILTER_REFERENCE_FRAME_LLH) != MIP_ACK_OK)
			PX4_ERR("ERROR: Could not set filter initialization configuration!");


	}
		break;

	default:
		break;
	}

	//
	// Setup the rotation based on PX4 standard rotation sets
	//

	if (mip_3dm_write_sensor_2_vehicle_transform_euler(&device, math::radians<float>(rot_lookup[_config.rot].roll),
			math::radians<float>(rot_lookup[_config.rot].pitch),
			math::radians<float>(rot_lookup[_config.rot].yaw)) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not set sensor-to-vehicle transformation!");
		return;
	}


	if (mip_3dm_write_datastream_control(&device, MIP_3DM_DATASTREAM_CONTROL_COMMAND_ALL_STREAMS, true) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not enable the data stream");
		return;
	}

	//
	//Resume the device
	//

	if (mip_base_resume(&device) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not resume the device!");
		return;
	}

	_is_initialized = true;

}

void CvIns::service_cv7()
{
	mip_interface_update(&device, false);

	switch (_config.selected_mode) {
	case mode_ahrs:
	case mode_imu:
	case mode_ins:
	{
		sensor_gps_s gps{0};
		// No new data
		if(!_sensor_gps_sub.update(&gps)){
			break;
		}

		// Fix isn't 3D or RTK or RTCM
		if((gps.fix_type < 3) || (gps.fix_type > 6)){
			break;
		}

		#define deg_conv(x) (double((x*1.0) / 10000000.0))

		// If the timestamp has not been set, then don't send any data
		// into the cv7 filter
		if(gps.time_utc_usec == 0){
			return;
		}

		mip_time t;
		t.timebase = MIP_TIME_TIMEBASE_TIME_OF_ARRIVAL;
		t.reserved = 0x01;
		t.nanoseconds = 0; // No offset


		// float llh_uncertainty[3] = {gps.eph,gps.eph,gps.eph}; // What is the uncertainty?
		float llh_uncertainty[3] = {1.0, 1.0, 1.0};
		mip_aiding_llh_pos(&device,&t,MIP_FILTER_REFERENCE_FRAME_LLH,deg_conv(gps.lat), deg_conv(gps.lon), ((gps.alt_ellipsoid*1.0)/1000.0),llh_uncertainty, MIP_AIDING_LLH_POS_COMMAND_VALID_FLAGS_ALL);

		if(gps.vel_ned_valid){
			float ned_v[3] = {gps.vel_n_m_s,gps.vel_e_m_s,gps.vel_d_m_s};
			// float ned_velocity_uncertainty[3] = {gps.s_variance_m_s,gps.s_variance_m_s,gps.s_variance_m_s}; // What is the uncertainty of NED velocity?
			float ned_velocity_uncertainty[3] = {0.1, 0.1, 0.1};
			mip_aiding_ned_vel(&device,&t,MIP_FILTER_REFERENCE_FRAME_LLH,ned_v, ned_velocity_uncertainty,MIP_AIDING_NED_VEL_COMMAND_VALID_FLAGS_ALL);
		}

		if(PX4_ISFINITE(gps.heading)){
			// float heading = PX4_ISFINITE(gps.heading_offset) ? gps.heading + gps.heading_offset : gps.heading;
			float heading = gps.heading;
			// There are no pre-defined flags for the heading (that I can find), setting everything to 1 for now
			mip_aiding_true_heading(&device,&t,MIP_FILTER_REFERENCE_FRAME_LLH,heading,gps.heading_accuracy,0xff);
		}
	}

	break;


	default:
		break;
	}
}

void CvIns::initialize_logger()
{
	if (_logger.is_initialized()) {
		return;
	}

	PX4_INFO("Creating the Logger");
	_logger.set_file_name("sess001.ulg");
	_logger.thread_start();
	PX4_INFO("Created the Logger");
}

template<typename T>
int CvIns::param_load(uint32_t idx, const char* type, T &val){
	char str[20] {};
	sprintf(str, "CAL_MAG%" PRIu32 "_%s", idx, type);
	val = 0;

	param_t param_handle = param_find_no_notification(str);

	if (param_handle == PARAM_INVALID) {
		return PX4_ERROR;
	}

	// find again and get value, but this time mark it active
	if (param_get(param_find(str), &val) != OK) {
		return PX4_ERROR;
	}
	return PX4_OK;
}

void CvIns::apply_mag_cal(){
	static constexpr int MAX_SENSOR_COUNT = 4;
	bool is_match = false;
	uint32_t mag_idx = 0;

	for (unsigned i = 0; i < MAX_SENSOR_COUNT; ++i) {

		int32_t device_id_val = 0;
		if(param_load<int32_t>(i,"ID",device_id_val) != PX4_OK){
			continue;
		}

		is_match = ((uint32_t)device_id_val == _config.device_id);

		if(is_match){
			mag_idx = i;
			break;
		}
	}

	// There is no match so no calibrations to write
	if(!is_match){
		PX4_INFO("Mag Cal couldn't find matching ID");
		return;
	}

	int32_t prio{0};

	if(param_load<int32_t>(mag_idx,"PRIO",prio) != PX4_OK){
		PX4_WARN("Mag Cal couldn't PRIO");
		return;
	}

	if(prio <= 0){
		PX4_INFO("Mag Cal for this device is disabled");
		return;
	}

	float hard_iron_offset[3] = {0};
	bool success = true;
	success |= param_load<float>(mag_idx,"XOFF",hard_iron_offset[0]) == PX4_OK;
	success |= param_load<float>(mag_idx,"YOFF",hard_iron_offset[1]) == PX4_OK;
	success |= param_load<float>(mag_idx,"ZOFF",hard_iron_offset[2]) == PX4_OK;
	// Failed getting the parameters, don't attempt to write anything
	if(!success){
		PX4_WARN("Mag Cal couldn't read hard iron offsets");
		return;
	}

	if (mip_3dm_write_mag_hard_iron_offset(&device, hard_iron_offset) != MIP_ACK_OK) {
		PX4_ERR("Mag Cal couldn't set the hard iron offset");
		return;
	}

	// [X  D1  D2]
	// [0  Y   D3]
	// [0  0   Z ]
	float soft_iron_matrix[9] = {0};
	float xcomp{0};  // Power compensation
	float ycomp{0};  // Power compensation
	float zcomp{0};  // Power compensation
	success |= param_load<float>(mag_idx,"XSCALE",soft_iron_matrix[0]) == PX4_OK;
	success |= param_load<float>(mag_idx,"YSCALE",soft_iron_matrix[4]) == PX4_OK;
	success |= param_load<float>(mag_idx,"ZSCALE",soft_iron_matrix[8]) == PX4_OK;
	success |= param_load<float>(mag_idx,"XODIAG",soft_iron_matrix[1]) == PX4_OK;
	success |= param_load<float>(mag_idx,"YODIAG",soft_iron_matrix[2]) == PX4_OK;
	success |= param_load<float>(mag_idx,"ZODIAG",soft_iron_matrix[5]) == PX4_OK;

	// TODO: How are the power compensation cal values used in the MIP SDK? I don't think they are
	success |= param_load<float>(mag_idx,"XCOMP",xcomp) == PX4_OK;
	success |= param_load<float>(mag_idx,"YCOMP",ycomp) == PX4_OK;
	success |= param_load<float>(mag_idx,"ZCOMP",zcomp) == PX4_OK;

	// Failed getting the parameters, don't attempt to write anything
	if(!success){
		PX4_WARN("Mag Cal couldn't read soft iron offsets");
		return;
	}

	if (mip_3dm_write_mag_soft_iron_matrix(&device, soft_iron_matrix) != MIP_ACK_OK) {
		PX4_ERR("Mag Cal couldn't set the soft iron offset");
		return;
	}

	char str[20] {};
	sprintf(str, "CAL_MAG%" PRIu32 "_PRIO", mag_idx);
	int32_t prio_disable_val = 0;
	if (param_set_no_notification(param_find(str), &prio_disable_val) != PX4_OK) {
		PX4_ERR("Mag Cal failed to disable PX4 internal mag cal");
		// TODO: Clear written values in the CV7?
	}

	PX4_INFO("Mag Cal finished writing calibrations");
}

void CvIns::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

#ifdef LOG_TRANSACTIONS
	initialize_logger();
#endif

	initialize_cv7();

	// Initialization failed, stop the module
	if (_is_init_failed) {
		request_stop();
		perf_end(_loop_perf);
		return;
	}

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)

		_delay_offset = _param_cv7_delay.get();

		// Disable this for now, work in progress
		// apply_mag_cal();
	}

	service_cv7();

	perf_end(_loop_perf);
}

int CvIns::task_spawn(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	const char *dev = "/dev/ttyS3";
	int32_t rot = ROTATION_NONE;

	while ((ch = px4_getopt(argc, argv, "d:r:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			dev = myoptarg;
			break;

		case 'r':
			rot = atoi(myoptarg);

			if (rot >= ROTATION_MAX) {
				rot = ROTATION_NONE;
			}

			break;
		}
	}

	if (dev == nullptr || strlen(dev) == 0) {
		print_usage("no device specified");
		_object.store(nullptr);
		_task_id = -1;

		return PX4_ERROR;
	}
	PX4_INFO("Opening device port %s",dev);
	CvIns *instance = new CvIns(dev, rot);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		// Get a local reference
		cv7_ins = instance;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int CvIns::print_status()
{
	PX4_INFO_RAW("Serial Port Open %d Handle %d Device %s\n", device_uart.is_open(), device_uart.uart_get_fd(),
		     _port);
	PX4_INFO_RAW("TX Bytes %lu\n", _debug_tx_bytes);
	PX4_INFO_RAW("Min %lu\n", _debug_rx_bytes[0]);
	PX4_INFO_RAW("Total %lu\n", _debug_rx_bytes[1]);
	PX4_INFO_RAW("Max %lu\n", _debug_rx_bytes[2]);
	PX4_INFO_RAW("Avg %f\n", static_cast<double>(_debug_rx_bytes[1] * 1.f / _debug_rx_bytes[3] * 1.f));
	_debug_rx_bytes[0] = UINT32_MAX;

	for (int i = 1; i < 4; i++) {
		_debug_rx_bytes[i] = 0;
	}
	_debug_tx_bytes = 0;

	#ifdef LOG_TRANSACTIONS
	PX4_INFO_RAW("MIP Stream Log \n");
	PX4_INFO_RAW("Logged TX %llu\n", _logger.logged_bytes[0]);
	PX4_INFO_RAW("Logged RX %llu\n", _logger.logged_bytes[1]);
	PX4_INFO_RAW("Overflow TX Counts %llu\n", _logger.buffer_full[0]);
	PX4_INFO_RAW("Overflow RX Counts %llu\n", _logger.buffer_full[1]);
	PX4_INFO_RAW("High Watermark TX %llu\n", _logger.buffer_high_watermark[0]);
	PX4_INFO_RAW("High Watermark RX %llu\n", _logger.buffer_high_watermark[1]);
	_logger.logged_bytes[0] = 0;
	_logger.logged_bytes[1] = 0;
	_logger.buffer_high_watermark[0] = 0;
	_logger.buffer_high_watermark[1] = 0;
	_logger.buffer_full[0] = 0;
	_logger.buffer_full[1] = 0;
	#endif
	if(_f_status.sample.filter_state == 4){
		PX4_INFO_RAW("Filter State FULL NAV\n");
	}
	if(_f_status.sample.filter_state == 3){
		PX4_INFO_RAW("Filter State AHRS\n");
	}
	if(_f_status.sample.filter_state == 2){
		PX4_INFO_RAW("Filter State VERT_GYRO\n");
	}
	if(_f_status.sample.filter_state == 1){
		PX4_INFO_RAW("Filter State INIT\n");
	}
	if(_f_status.sample.filter_state == 0){
		PX4_INFO_RAW("Filter State UNKNOWN\n");
	}
	// Mask the lower three bits
	PX4_INFO_RAW("Filter Status 0x%X\n",_f_status.sample.status_flags);
	uint8_t filt_status = _f_status.sample.status_flags & 0x0003;
	if(filt_status == 0x01){
		PX4_INFO_RAW("Filter Status STABLE\n");
	} else if(filt_status == 0x02){
		PX4_INFO_RAW("Filter Status CONVERGING\n");
	} else if(filt_status == 0x03){
		PX4_INFO_RAW("Filter Status UNSTABLE/RECOVERING\n");
	} else {
		PX4_INFO_RAW("Filter Status UNKNOWN\n");
	}

	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int CvIns::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CvIns::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
CV7 IMU Driver.

Communicates over serial port an utilizes the manufacturer provided MIP SDK.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("cv7_ins", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS2", "<file:dev>", "CV7 Port", true);
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, 0, ROTATION_MAX, "See enum Rotation for values", true);

	return 0;
}

extern "C" __EXPORT int cv7_ins_main(int argc, char *argv[])
{
	return CvIns::main(argc, argv);
}