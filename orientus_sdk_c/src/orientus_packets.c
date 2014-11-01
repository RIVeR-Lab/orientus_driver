/****************************************************************/
/*                                                              */
/*         Advanced Navigation Packet Protocol Library          */
/*         C Language Dynamic Orientus SDK, Version 1.1         */
/*   Copyright 2013, Xavier Orr, Advanced Navigation Pty Ltd    */
/*                                                              */
/****************************************************************/
/*
 * Copyright (C) 2013 Advanced Navigation Pty Ltd
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
 
 #include <stdint.h>
 #include <string.h>
 #include "an_packet_protocol.h"
 #include "orientus_packets.h"

/*
 * This file contains functions to decode and encode packets
 *
 * Decode functions take an an_packet_t and turn it into a type specific
 * to that packet so that the fields can be conveniently accessed. Decode
 * functions return 0 for success and 1 for failure. Decode functions are
 * used when receiving packets.
 *
 * Example decode
 *
 * an_packet_t an_packet
 * acknowledge_packet_t acknowledge_packet
 * ...
 * decode_acknowledge_packet(&acknowledge_packet, &an_packet);
 * printf("acknowledge id %d with result %d\n", acknowledge_packet.packet_id, acknowledge_packet.acknowledge_result);
 *
 * Encode functions take a type specific structure and turn it into an
 * an_packet_t. Encode functions are used when sending packets. Don't
 * forget to free the returned packet with an_packet_free().
 *
 * Example encode
 *
 * an_packet_t *an_packet;
 * boot_mode_packet_t boot_mode_packet;
 * ...
 * boot_mode_packet.boot_mode = boot_mode_bootloader;
 * an_packet = encode_boot_mode_packet(&boot_mode_packet);
 * serial_port_transmit(an_packet_pointer(an_packet), an_packet_size(an_packet));
 * an_packet_free(&an_packet);
 *
 */

int decode_acknowledge_packet(acknowledge_packet_t *acknowledge_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_acknowledge && an_packet->length == 4)
	{
		acknowledge_packet->packet_id = an_packet->data[0];
		memcpy(&acknowledge_packet->packet_crc, &an_packet->data[1], sizeof(uint16_t));
		acknowledge_packet->acknowledge_result = an_packet->data[3];
		return 0;
	}
	else return 1;
}

an_packet_t *encode_request_packet(uint8_t requested_packet_id)
{
	an_packet_t *an_packet = an_packet_allocate(1, packet_id_request);
	an_packet->data[0] = requested_packet_id;
	return an_packet;
}

int decode_boot_mode_packet(boot_mode_packet_t *boot_mode_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_boot_mode && an_packet->length == 1)
	{
		boot_mode_packet->boot_mode = an_packet->data[0];
		return 0;
	}
	else return 1;
}

an_packet_t *encode_boot_mode_packet(boot_mode_packet_t *boot_mode_packet)
{
	an_packet_t *an_packet = an_packet_allocate(1, packet_id_boot_mode);
	if(an_packet != NULL)
	{
		an_packet->data[0] = boot_mode_packet->boot_mode;
	}
	return an_packet;
}

int decode_device_information_packet(device_information_packet_t *device_information_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_device_information && an_packet->length == 24)
	{
		memcpy(&device_information_packet->software_version, &an_packet->data[0], sizeof(uint32_t));
		memcpy(&device_information_packet->device_id, &an_packet->data[4], sizeof(uint32_t));
		memcpy(&device_information_packet->hardware_revision, &an_packet->data[8], sizeof(uint32_t));
		memcpy(&device_information_packet->serial_number[0], &an_packet->data[12], 3*sizeof(uint32_t));
		return 0;
	}
	else return 1;
}

an_packet_t *encode_restore_factory_settings_packet()
{
	uint32_t verification = 0x85429E1C;
	an_packet_t *an_packet = an_packet_allocate(4, packet_id_restore_factory_settings);
	if(an_packet != NULL)
	{
		memcpy(&an_packet->data[0], &verification, sizeof(uint32_t));
	}
	return an_packet;
}

an_packet_t *encode_reset_packet()
{
	uint32_t verification = 0x21057A7E;
	an_packet_t *an_packet = an_packet_allocate(4, packet_id_reset);
	if(an_packet != NULL)
	{
		memcpy(&an_packet->data[0], &verification, sizeof(uint32_t));
	}
	return an_packet;
}
 
int decode_system_state_packet(system_state_packet_t *system_state_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_system_state && an_packet->length == 100)
	{
		memcpy(&system_state_packet->system_status, &an_packet->data[0], sizeof(uint16_t));
		memcpy(&system_state_packet->filter_status, &an_packet->data[2], sizeof(uint16_t));
		memcpy(&system_state_packet->unix_time_seconds, &an_packet->data[4], sizeof(uint32_t));
		memcpy(&system_state_packet->microseconds, &an_packet->data[8], sizeof(uint32_t));
		memcpy(&system_state_packet->orientation[0], &an_packet->data[64], 3 * sizeof(float));
		memcpy(&system_state_packet->angular_velocity[0], &an_packet->data[76], 3 * sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_unix_time_packet(unix_time_packet_t *unix_time_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_unix_time && an_packet->length == 8)
	{
		memcpy(&unix_time_packet->unix_time_seconds, &an_packet->data[0], sizeof(uint32_t));
		memcpy(&unix_time_packet->microseconds, &an_packet->data[4], sizeof(uint32_t));
		return 0;
	}
	else return 1;
}

int decode_status_packet(status_packet_t *status_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_status && an_packet->length == 4)
	{
		memcpy(&status_packet->system_status, &an_packet->data[0], sizeof(uint16_t));
		memcpy(&status_packet->filter_status, &an_packet->data[2], sizeof(uint16_t));
		return 0;
	}
	else return 1;
}

int decode_euler_orientation_standard_deviation_packet(euler_orientation_standard_deviation_packet_t *euler_orientation_standard_deviation, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_euler_orientation_standard_deviation && an_packet->length == 12)
	{
		memcpy(&euler_orientation_standard_deviation->standard_deviation[0], &an_packet->data[0], 3*sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_quaternion_orientation_standard_deviation_packet(quaternion_orientation_standard_deviation_packet_t *quaternion_orientation_standard_deviation_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_quaternion_orientation_standard_deviation && an_packet->length == 16)
	{
		memcpy(&quaternion_orientation_standard_deviation_packet->standard_deviation[0], &an_packet->data[0], 4*sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_raw_sensors_packet(raw_sensors_packet_t *raw_sensors_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_raw_sensors && an_packet->length == 48)
	{
		memcpy(&raw_sensors_packet->accelerometers[0], &an_packet->data[0], 3 * sizeof(float));
		memcpy(&raw_sensors_packet->gyroscopes[0], &an_packet->data[12], 3 * sizeof(float));
		memcpy(&raw_sensors_packet->magnetometers[0], &an_packet->data[24], 3 * sizeof(float));
		memcpy(&raw_sensors_packet->imu_temperature, &an_packet->data[36], sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_euler_orientation_packet(euler_orientation_packet_t *euler_orientation_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_euler_orientation && an_packet->length == 12)
	{
		memcpy(&euler_orientation_packet->orientation[0], &an_packet->data[0], 3*sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_quaternion_orientation_packet(quaternion_orientation_packet_t *quaternion_orientation_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_quaternion_orientation && an_packet->length == 16)
	{
		memcpy(&quaternion_orientation_packet->orientation[0], &an_packet->data[0], 4*sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_dcm_orientation_packet(dcm_orientation_packet_t *dcm_orientation_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_dcm_orientation && an_packet->length == 36)
	{
		memcpy(&dcm_orientation_packet->orientation[0][0], &an_packet->data[0], 9*sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_angular_velocity_packet(angular_velocity_packet_t *angular_velocity_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_angular_velocity && an_packet->length == 12)
	{
		memcpy(&angular_velocity_packet->angular_velocity[0], &an_packet->data[0], 3*sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_angular_acceleration_packet(angular_acceleration_packet_t *angular_acceleration_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_angular_acceleration && an_packet->length == 12)
	{
		memcpy(&angular_acceleration_packet->angular_acceleration[0], &an_packet->data[0], 3*sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_external_position_velocity_packet(external_position_velocity_packet_t *external_position_velocity_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_external_position_velocity && an_packet->length == 60)
	{
		memcpy(&external_position_velocity_packet->position[0], &an_packet->data[0], 3*sizeof(double));
		memcpy(&external_position_velocity_packet->velocity[0], &an_packet->data[24], 3*sizeof(float));
		memcpy(&external_position_velocity_packet->position_standard_deviation[0], &an_packet->data[36], 3*sizeof(float));
		memcpy(&external_position_velocity_packet->velocity_standard_deviation[0], &an_packet->data[48], 3*sizeof(float));
		return 0;
	}
	else return 1;
}

an_packet_t *encode_external_position_velocity_packet(external_position_velocity_packet_t *external_position_velocity_packet)
{
	an_packet_t *an_packet = an_packet_allocate(60, packet_id_external_position_velocity);
	if(an_packet != NULL)
	{
		memcpy(&an_packet->data[0], &external_position_velocity_packet->position[0], 3*sizeof(double));
		memcpy(&an_packet->data[24], &external_position_velocity_packet->velocity[0], 3*sizeof(float));
		memcpy(&an_packet->data[36], &external_position_velocity_packet->position_standard_deviation[0], 3*sizeof(float));
		memcpy(&an_packet->data[48], &external_position_velocity_packet->velocity_standard_deviation[0], 3*sizeof(float));
	}
	return an_packet;
}

int decode_external_position_packet(external_position_packet_t *external_position_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_external_position && an_packet->length == 36)
	{
		memcpy(&external_position_packet->position[0], &an_packet->data[0], 3*sizeof(double));
		memcpy(&external_position_packet->standard_deviation[0], &an_packet->data[24], 3*sizeof(float));
		return 0;
	}
	else return 1;
}

an_packet_t *encode_external_position_packet(external_position_packet_t *external_position_packet)
{
	an_packet_t *an_packet = an_packet_allocate(36, packet_id_external_position);
	if(an_packet != NULL)
	{
		memcpy(&an_packet->data[0], &external_position_packet->position[0], 3*sizeof(double));
		memcpy(&an_packet->data[24], &external_position_packet->standard_deviation[0], 3*sizeof(float));
	}
	return an_packet;
}

int decode_external_velocity_packet(external_velocity_packet_t *external_velocity_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_external_velocity && an_packet->length == 24)
	{
		memcpy(&external_velocity_packet->velocity[0], &an_packet->data[0], 3*sizeof(float));
		memcpy(&external_velocity_packet->standard_deviation[0], &an_packet->data[12], 3*sizeof(float));
		return 0;
	}
	else return 1;
}

an_packet_t *encode_external_velocity_packet(external_velocity_packet_t *external_velocity_packet)
{
	an_packet_t *an_packet = an_packet_allocate(24, packet_id_external_velocity);
	if(an_packet != NULL)
	{
		memcpy(&an_packet->data[0], &external_velocity_packet->velocity[0], 3*sizeof(float));
		memcpy(&an_packet->data[12], &external_velocity_packet->standard_deviation[0], 3*sizeof(float));
	}
	return an_packet;
}

int decode_external_heading_packet(external_heading_packet_t *external_heading_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_external_heading && an_packet->length == 8)
	{
		memcpy(&external_heading_packet->heading, &an_packet->data[0], sizeof(float));
		memcpy(&external_heading_packet->standard_deviation, &an_packet->data[4], sizeof(float));
		return 0;
	}
	else return 1;
}

an_packet_t *encode_external_heading_packet(external_heading_packet_t *external_heading_packet)
{
	an_packet_t *an_packet = an_packet_allocate(8, packet_id_external_heading);
	if(an_packet != NULL)
	{
		memcpy(&an_packet->data[0], &external_heading_packet->heading, sizeof(float));
		memcpy(&an_packet->data[4], &external_heading_packet->standard_deviation, sizeof(float));
	}
	return an_packet;
}

int decode_running_time_packet(running_time_packet_t *running_time_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_running_time && an_packet->length == 8)
	{
		memcpy(&running_time_packet->seconds, &an_packet->data[0], sizeof(uint32_t));
		memcpy(&running_time_packet->microseconds, &an_packet->data[4], sizeof(uint32_t));
		return 0;
	}
	else return 1;
}

int decode_local_magnetics_packet(local_magnetics_packet_t *local_magnetics_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_local_magnetics && an_packet->length == 12)
	{
		memcpy(&local_magnetics_packet->magnetic_field[0], &an_packet->data[0], 3*sizeof(float));
		return 0;
	}
	else return 1;
}

int decode_packet_timer_period_packet(packet_timer_period_packet_t *packet_timer_period_packet, an_packet_t *an_packet)
{
    if(an_packet->id == packet_id_packet_timer_period && an_packet->length == 4)
    {
        packet_timer_period_packet->permanent = an_packet->data[0];
        packet_timer_period_packet->utc_synchronisation = an_packet->data[1];
        memcpy(&packet_timer_period_packet->packet_timer_period, &an_packet->data[2], sizeof(uint16_t));
        return 0;
    }
    else return 1;
}

an_packet_t *encode_packet_timer_period_packet(packet_timer_period_packet_t *packet_timer_period_packet)
{
	an_packet_t *an_packet = an_packet_allocate(4, packet_id_packet_timer_period);
	if(an_packet != NULL)
	{
		an_packet->data[0] = packet_timer_period_packet->permanent > 0;
		an_packet->data[1] = packet_timer_period_packet->utc_synchronisation > 0;
		memcpy(&an_packet->data[2], &packet_timer_period_packet->packet_timer_period, sizeof(uint16_t));
	}
    return an_packet;
}

int decode_packet_periods_packet(packet_periods_packet_t *packet_periods_packet, an_packet_t *an_packet)
{
    if(an_packet->id == packet_id_packet_periods && (an_packet->length - 2) % 5 == 0)
    {
        int i;
        int packet_periods_count = (an_packet->length - 2) / 5;
        packet_periods_packet->permanent = an_packet->data[0];
        packet_periods_packet->clear_existing_packets = an_packet->data[1];
        for(i = 0; i < MAXIMUM_PACKET_PERIODS; i++)
        {
            if(i < packet_periods_count)
            {
                packet_periods_packet->packet_periods[i].packet_id = an_packet->data[2 + 5*i];
                memcpy(&packet_periods_packet->packet_periods[i].period, &an_packet->data[2 + 5*i + 1], sizeof(uint32_t));
            }
            else memset(&packet_periods_packet->packet_periods[i], 0, sizeof(packet_period_t));
        }
        return 0;
    }
    else return 1;
}

an_packet_t *encode_packet_periods_packet(packet_periods_packet_t *packet_periods_packet)
{
    int i;
    an_packet_t *an_packet = an_packet_allocate(252, packet_id_packet_periods);
	if(an_packet != NULL)
	{
		an_packet->data[0] = packet_periods_packet->permanent > 0;
		an_packet->data[1] = packet_periods_packet->clear_existing_packets;
		for(i = 0; i < MAXIMUM_PACKET_PERIODS; i++)
		{
			if(packet_periods_packet->packet_periods[i].packet_id)
			{
				an_packet->data[2 + 5*i] = packet_periods_packet->packet_periods[i].packet_id;
				memcpy(&an_packet->data[2 + 5*i + 1], &packet_periods_packet->packet_periods[i].period, sizeof(uint32_t));
			}
			else break;
		}
		an_packet->length = 2 + 5*i;
	}
    return an_packet;
}

int decode_baud_rates_packet(baud_rates_packet_t *baud_rates_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_baud_rates && an_packet->length == 17)
	{
		baud_rates_packet->permanent = an_packet->data[0];
		memcpy(&baud_rates_packet->primary_baud_rate, &an_packet->data[1], sizeof(uint32_t));
		memcpy(&baud_rates_packet->gpio_1_2_baud_rate, &an_packet->data[5], sizeof(uint32_t));
		memcpy(&baud_rates_packet->gpio_3_4_baud_rate, &an_packet->data[9], sizeof(uint32_t));
		return 0;
	}
	else return 1;
}

an_packet_t *encode_baud_rates_packet(baud_rates_packet_t *baud_rates_packet)
{
	an_packet_t *an_packet = an_packet_allocate(17, packet_id_baud_rates);
	if(an_packet != NULL)
	{
		an_packet->data[0] = baud_rates_packet->permanent;
		memcpy(&an_packet->data[1], &baud_rates_packet->primary_baud_rate, sizeof(uint32_t));
		memcpy(&an_packet->data[5], &baud_rates_packet->gpio_1_2_baud_rate, sizeof(uint32_t));
		memcpy(&an_packet->data[9], &baud_rates_packet->gpio_3_4_baud_rate, sizeof(uint32_t));
		memset(&an_packet->data[13], 0, sizeof(uint32_t));
	}
	return an_packet;
}

int decode_sensor_ranges_packet(sensor_ranges_packet_t *sensor_ranges_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_sensor_ranges && an_packet->length == 4)
	{
		memcpy(sensor_ranges_packet, an_packet->data, 4*sizeof(uint8_t));
		return 0;
	}
	else return 1;
}

an_packet_t *encode_sensor_ranges_packet(sensor_ranges_packet_t *sensor_ranges_packet)
{
	an_packet_t *an_packet = an_packet_allocate(4, packet_id_sensor_ranges);
	if(an_packet != NULL)
	{
		memcpy(an_packet->data, sensor_ranges_packet, 4*sizeof(uint8_t));
	}
	return an_packet;
}

int decode_installation_alignment_packet(installation_alignment_packet_t *installation_alignment_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_installation_alignment && an_packet->length == 73)
	{
		installation_alignment_packet->permanent = an_packet->data[0];
		memcpy(&installation_alignment_packet->alignment_dcm[0][0], &an_packet->data[1], 9*sizeof(float));
		return 0;
	}
	else return 1;
}

an_packet_t *encode_installation_alignment_packet(installation_alignment_packet_t *installation_alignment_packet)
{
	an_packet_t *an_packet = an_packet_allocate(73, packet_id_installation_alignment);
	if(an_packet != NULL)
	{
		an_packet->data[0] = installation_alignment_packet->permanent;
		memcpy(&an_packet->data[1], &installation_alignment_packet->alignment_dcm[0][0], 9*sizeof(float));
		memset(&an_packet->data[37], 0, 9*sizeof(float));
	}
	return an_packet;
}

int decode_filter_options_packet(filter_options_packet_t *filter_options_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_filter_options && an_packet->length == 17)
	{
		filter_options_packet->permanent = an_packet->data[0];
		filter_options_packet->vehicle_type = an_packet->data[1];
		filter_options_packet->magnetometers_enabled = an_packet->data[3];
		filter_options_packet->velocity_heading_enabled = an_packet->data[5];
		return 0;
	}
	else return 1;
}

an_packet_t *encode_filter_options_packet(filter_options_packet_t *filter_options_packet)
{
	an_packet_t *an_packet = an_packet_allocate(17, packet_id_filter_options);
	if(an_packet != NULL)
	{
		memset(an_packet->data, 0, 17*sizeof(uint8_t));
		an_packet->data[0] = filter_options_packet->permanent;
		an_packet->data[1] = filter_options_packet->vehicle_type;
		an_packet->data[3] = filter_options_packet->magnetometers_enabled;
		an_packet->data[5] = filter_options_packet->velocity_heading_enabled;
	}
	return an_packet;
}

int decode_gpio_configuration_packet(gpio_configuration_packet_t *gpio_configuration_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_gpio_configuration && an_packet->length == 13)
	{
		memcpy(gpio_configuration_packet, &an_packet->data[0], 5*sizeof(uint8_t));
		return 0;
	}
	else return 1;
}

an_packet_t *encode_gpio_configuration_packet(gpio_configuration_packet_t *gpio_configuration_packet)
{
	an_packet_t *an_packet = an_packet_allocate(13, packet_id_gpio_configuration);
	if(an_packet != NULL)
	{
		memcpy(&an_packet->data[0], gpio_configuration_packet, 5*sizeof(uint8_t));
		memset(&an_packet->data[5], 0, 8*sizeof(uint8_t));
	}
	return an_packet;
}

int decode_magnetic_calibration_values_packet(magnetic_calibration_values_packet_t *magnetic_calibration_values_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_magnetic_calibration_values && an_packet->length == 49)
	{
		magnetic_calibration_values_packet->permanent = an_packet->data[0];
		memcpy(magnetic_calibration_values_packet->hard_iron, &an_packet->data[5], 3*sizeof(float));
		memcpy(magnetic_calibration_values_packet->soft_iron, &an_packet->data[13], 9*sizeof(float));
		return 0;
	}
	else return 1;
}

an_packet_t *encode_magnetic_calibration_values_packet(magnetic_calibration_values_packet_t *magnetic_calibration_values_packet)
{
	an_packet_t *an_packet = an_packet_allocate(49, packet_id_magnetic_calibration_values);
	if(an_packet != NULL)
	{
		an_packet->data[0] = magnetic_calibration_values_packet->permanent;
		memcpy(&an_packet->data[1], magnetic_calibration_values_packet->hard_iron, 3*sizeof(float));
		memcpy(&an_packet->data[13], magnetic_calibration_values_packet->soft_iron, 9*sizeof(float));
	}
	return an_packet;
}

an_packet_t *encode_magnetic_calibration_configuration_packet(magnetic_calibration_configuration_packet_t *magnetic_calibration_configuration_packet)
{
	an_packet_t *an_packet = an_packet_allocate(1, packet_id_magnetic_calibration_configuration);
	if(an_packet != NULL)
	{
		an_packet->data[0] = magnetic_calibration_configuration_packet->magnetic_calibration_action;
	}
	return an_packet;
}

int decode_magnetic_calibration_status_packet(magnetic_calibration_status_packet_t *magnetic_calibration_status_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_magnetic_calibration_status && an_packet->length == 3)
	{
		magnetic_calibration_status_packet->magnetic_calibration_status = an_packet->data[0];
		magnetic_calibration_status_packet->magnetic_calibration_progress_percentage = an_packet->data[1];
		magnetic_calibration_status_packet->local_magnetic_error_percentage = an_packet->data[2];
		return 0;
	}
	return 1;
}

an_packet_t *encode_zero_alignment_packet(zero_alignment_packet_t *zero_alignment_packet)
{
	an_packet_t *an_packet = an_packet_allocate(1, packet_id_zero_alignment);
	if(an_packet != NULL)
	{
		an_packet->data[0] = zero_alignment_packet->permanent;
	}
	return an_packet;
}




int decode_acceleration_packet(acceleration_packet_t *acceleration_packet, an_packet_t *an_packet)
{
  if(an_packet->id == packet_id_acceleration && an_packet->length == 12)
    {
      memcpy(&acceleration_packet->acceleration[0], &an_packet->data[0], 3*sizeof(float));
      return 0;

    }
  else return 1;
}
