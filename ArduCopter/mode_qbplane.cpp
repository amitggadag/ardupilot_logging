#include "Copter.h"
#include <RC_Channel/RC_Channel.h>
#include "mode.h"

// QBPLANE mode: customized Quad-Biplane mode
// mode number: 29
void ModeQbplane::run()
{
	// convert the input to the desired body frame rate
	float target_roll, target_pitch, target_yaw;
	// Channel 7: switch roll & yaw inputs -> SB
	// PWM 988, switch up, Forward
	// PWM 1500, switch neutral, VTOL
	// PWM 2021, switch down, VTOL
	RC_Channel *cvft = rc().channel(int8_t(6)); // channel 7
	int16_t vft;
	vft = cvft->get_radio_in();

	if (vft < 1200) { // input from yaw stick -> roll; input from roll stick -> yaw FORWARD
		get_pilot_desired_angle_rates(channel_yaw->norm_input_dz(), channel_pitch->norm_input_dz(), channel_roll->norm_input_dz(), target_roll, target_pitch, target_yaw);
	}
	else { // HOVER
		get_pilot_desired_angle_rates(channel_roll->norm_input_dz(), channel_pitch->norm_input_dz(), channel_yaw->norm_input_dz(), target_roll, target_pitch, target_yaw);
	}

	if (!motors->armed()) {
		// Motors should be Stopped
		motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
	} else if (copter.ap.throttle_zero
			   || (copter.air_mode == AirMode::AIRMODE_ENABLED && motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN)) {
		// throttle_zero is never true in air mode, but the motors should be allowed to go through ground idle
		// in order to facilitate the spoolup block

		// Attempting to Land or motors not yet spinning
		// if airmode is enabled only an actual landing will spool down the motors
		motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
	} else {
		motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
	}

	float pilot_desired_throttle = get_pilot_desired_throttle();

	switch (motors->get_spool_state()) {
	case AP_Motors::SpoolState::SHUT_DOWN:
		// Motors Stopped
		attitude_control->reset_target_and_rate(true);
		attitude_control->reset_rate_controller_I_terms();
		pilot_desired_throttle = 0.0f;
		break;

	case AP_Motors::SpoolState::GROUND_IDLE:
		// Landed
		attitude_control->reset_target_and_rate();
		attitude_control->reset_rate_controller_I_terms_smoothly();
		pilot_desired_throttle = 0.0f;
		break;

	case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
		// clear landing flag above zero throttle
		if (!motors->limit.throttle_lower) {
			set_land_complete(false);
		}
		break;

	case AP_Motors::SpoolState::SPOOLING_UP:
	case AP_Motors::SpoolState::SPOOLING_DOWN:
		// do nothing
		break;
	}

	// run attitude controller
	if (g2.acro_options.get() & uint8_t(AcroOptions::RATE_LOOP_ONLY)) {
		if (vft < 1200) { // input from yaw stick -> roll; input from roll stick -> yaw FORWARD
			attitude_control->input_rate_bf_roll_pitch_yaw_2(target_roll, target_pitch, -1.0f*target_yaw);
		}
		else { // HOVER
			attitude_control->input_rate_bf_roll_pitch_yaw_2(target_roll, target_pitch, target_yaw);
		}
	} else {
		attitude_control->input_rate_bf_roll_pitch_yaw(target_roll, target_pitch, target_yaw);
	}

	// output pilot's throttle without angle boost
	attitude_control->set_throttle_out(pilot_desired_throttle, false, copter.g.throttle_filt);
}

bool ModeQbplane::init(bool ignore_checks)
{
	if (g2.acro_options.get() & uint8_t(AcroOptions::AIR_MODE)) {
		disable_air_mode_reset = false;
		copter.air_mode = AirMode::AIRMODE_ENABLED;
	}

	return true;
}

void ModeQbplane::exit()
{
	if (!disable_air_mode_reset && (g2.acro_options.get() & uint8_t(AcroOptions::AIR_MODE))) {
		copter.air_mode = AirMode::AIRMODE_DISABLED;
	}
	disable_air_mode_reset = false;
}

void ModeQbplane::air_mode_aux_changed()
{
	disable_air_mode_reset = true;
}

float ModeQbplane::throttle_hover() const
{
	if (g2.acro_thr_mid > 0) {
		return g2.acro_thr_mid;
	}
	return Mode::throttle_hover();
}

// get_pilot_desired_angle_rates - transform pilot's normalised roll pitch and yaw input into a desired lean angle rates
// inputs are -1 to 1 and the function returns desired angle rates in centi-degrees-per-second
void ModeQbplane::get_pilot_desired_angle_rates(float roll_in, float pitch_in, float yaw_in, float &roll_out, float &pitch_out, float &yaw_out)
{
	Vector3f rate_bf_request_cd;
	// apply circular limit to pitch and roll inputs
	float total_in = norm(pitch_in, roll_in);
	if (total_in > 1.0) {
		float ratio = 1.0 / total_in;
		roll_in *= ratio;
		pitch_in *= ratio;
	}

	// calculate roll, pitch rate requests
	// roll expo
	rate_bf_request_cd.x = g2.command_model_acro_rp.get_rate() * 100.0 * input_expo(roll_in, g2.command_model_acro_rp.get_expo());
	// pitch expo
	rate_bf_request_cd.y = g2.command_model_acro_rp.get_rate() * 100.0 * input_expo(pitch_in, g2.command_model_acro_rp.get_expo());
	// yaw expo
	rate_bf_request_cd.z = g2.command_model_acro_y.get_rate() * 100.0 * input_expo(yaw_in, g2.command_model_acro_y.get_expo());

	// hand back rate request
	roll_out = rate_bf_request_cd.x;
	pitch_out = rate_bf_request_cd.y;
	yaw_out = rate_bf_request_cd.z;
}
