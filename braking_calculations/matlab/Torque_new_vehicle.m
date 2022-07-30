g = 9.8;
pi = 3.14;
mass_ATV_kg = 200;
mass_driver = 70;
max_speed_kmph = 40;
max_speed_mps = 11.11;
mu_tyretoroad = 0.7;               % static cofficient 
mu_tyretoroad_kinetic = 0.6;        % kinetic cofficient
mu_pad=0.4;
h_cog= 0.450;
wheelbase= 1.450;
static_weight_ratio = 35/65;        % Front:Rear
wheel_radius_front=0.2667;
wheel_radius_rear=0.2921;
pedal_ratio = 6.1;
pedal_leverage=0.9;
man_pedal_force = 300;
front_disc_dia = 170*0.001;
rear_disc_dia = 190*0.001;
dia_eff_front = (front_disc_dia- 40*0.001);
dia_eff_rear = (rear_disc_dia - 20*0.001);
caliper_piston_dia_front = 1.25*25.4*0.001;
caliper_piston_dia_rear = 1.38*25.4*0.001;
%master cylinder required values
    tmc_piston_dia = (15.87)*0.001;
    area_tmc = ((tmc_piston_dia)^(2))*pi*0.25;
    stroke_mm = 33.274;
    stroke = 2.03327;
    
%fliud volume clearance required
    fluid_volume_running_clearance = area_tmc*stroke;
    fluid_volume_running_clearance_centicube = fluid_volume_running_clearance*(10^(6));

%rotational mass considerations

    %suspension
    rotational_mass_suspension_front = (1.97789);
    rotational_mass_suspension_rear = (2.16773);
    rotational_mass_suspension = rotational_mass_suspension_rear + rotational_mass_suspension_front;

    %transmission
    rotational_mass_gearbox_axles = 0.02399661475;
    rotational_mass_secondary_cvt = 0.02289715719;
    rotational_mass_transmission = rotational_mass_secondary_cvt + rotational_mass_gearbox_axles;
    rotational_mass = rotational_mass_suspension + rotational_mass_transmission;

%`average deceleration for whole stop
    decelration_mffd = mu_tyretoroad*g;
    x = max_speed_mps;
    y = ((max_speed_mps/(mu_tyretoroad_kinetic*g)) + 0.3);
    deceleration_average = x/y;

%stopping distance
    stopping_distance = (max_speed_mps^(2))/(2*deceleration_average);

%stopping time
    stopping_time = max_speed_mps/deceleration_average;

%angular acceleration of rotating tyre system
    angular_velocity_rear = max_speed_mps/wheel_radius_rear;
    angular_acceleration_rear = angular_velocity_rear/stopping_time;
    angular_velocity_front = max_speed_mps/wheel_radius_front;
    angular_acceleration_front = angular_velocity_front/stopping_time;
    
    angular_velocity_secondary_cvt = 342.348;
    angular_acceleration_secondary_cvt = angular_velocity_secondary_cvt/stopping_time;

%inertia torque calculations 
%suspension
    inertial_torque_suspension_front = rotational_mass_suspension_front*angular_acceleration_front;
    inertial_torque_suspension_rear = rotational_mass_suspension_rear*angular_acceleration_rear;
    %transmission
    inertial_torque_gearbox_axles = rotational_mass_gearbox_axles*angular_acceleration_rear;
    inertial_torque_secondary_cvt = rotational_mass_secondary_cvt*angular_acceleration_secondary_cvt;
    inertial_torque_transmission_total = inertial_torque_gearbox_axles + inertial_torque_secondary_cvt;
    
%weight transfer calculations
    weight_total = (mass_ATV_kg + mass_driver)*1*9.8;  
    weight_transfer = (mu_tyretoroad*h_cog*weight_total)/wheelbase ;
    weight_front= weight_total*((((static_weight_ratio)^(-1))+1)^(-1))+weight_transfer;
    weight_front_one_wheel= weight_front/2;   %on one wheel
    weight_rear = (weight_total - weight_front);

%brake force calculations
    brake_force_front_tyre= mu_tyretoroad*weight_front_one_wheel;
    brake_force_rear_tyre = mu_tyretoroad*weight_rear;

%brake torque calculations
    brake_torque_front=((brake_force_front_tyre)*wheel_radius_front) +(inertial_torque_suspension_front/2);
    brake_torque_rear=((brake_force_rear_tyre)*wheel_radius_rear) + (inertial_torque_suspension_rear/2) + inertial_torque_transmission_total;
    disp("Brake torque front:" + brake_torque_front);
    disp("Brake torque rear:" + brake_torque_rear);

    force_tmc = pedal_ratio*pedal_leverage*man_pedal_force;
    brake_line_pressure = force_tmc/area_tmc;

    area_caliper_piston_front = pi*((caliper_piston_dia_front)^2)*0.25;
    area_caliper_piston_rear = pi*((caliper_piston_dia_rear)^2)*0.25;

    caliper_force_rear = 2*brake_line_pressure*area_caliper_piston_rear;
    caliper_force_front = 2*brake_line_pressure*area_caliper_piston_front;

    brake_torque_by_caliper_front = caliper_force_front*mu_pad*dia_eff_front/2;
    brake_torque_by_caliper_rear = caliper_force_rear*mu_pad*dia_eff_rear/2;
    disp("brake_torque_by_caliper_front:" + brake_torque_by_caliper_front);
    disp("brake_torque_by_caliper_rear:" + brake_torque_by_caliper_rear);
%master cylinder losses( in centimeter cube)
 


%energy analysis
    kinetic_energy = (mass_ATV_kg + mass_driver)*(max_speed_mps^(2))/2;
    
    %suspension energy
        suspension_inertial_energy_front = rotational_mass_suspension_front*(angular_velocity_front^(2));
        suspension_inertial_energy_rear = rotational_mass_suspension_rear*(angular_velocity_rear^(2));
        suspension_inertial_energy = suspension_inertial_energy_front + suspension_inertial_energy_rear;

    %transmission energy
        transmission_inertial_energy = 0;
    
    %total energy
        total_energy = suspension_inertial_energy + transmission_inertial_energy + kinetic_energy;

    %average power
        average_power = total_energy/stopping_time;
    
    %energy discipated by brake pad and rotor interface
        system_response_time = 0.5;
        delta_theta_front = (angular_velocity_front*system_response_time) + (angular_acceleration_front*(system_response_time ^(2)))/2;
        energy_discipated_padrotor_front = 2*(mu_pad*caliper_force_front*delta_theta_front*dia_eff_front/2);

        energy_stopping_distance = (kinetic_energy - energy_discipated_padrotor_front)/((mass_ATV_kg + mass_driver)*g*mu_tyretoroad_kinetic);

%thermal considerations
    %heat flux   
    pad_outer_radius_front = front_disc_dia;
    pad_inner_radius_front = pad_outer_radius_front - 20*0.001;
    area_of_disc_rubbing_front = pi*((pad_outer_radius_front^(2)) - (pad_inner_radius_front^(2)));
    heat_flux = (energy_discipated_padrotor_front)/(system_response_time*area_of_disc_rubbing_front);