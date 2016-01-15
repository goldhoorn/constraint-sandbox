require 'orocos'
require 'roby'
require 'syskit'
Roby.app.robot 'avalon'
Roby.app.using_plugins 'syskit'
Syskit.conf.only_load_models = true
Syskit.conf.disables_local_process_server = true
Roby.app.single = true
Syskit.conf.disable_logging
Syskit.conf.disable_conf_logging
Roby.app.setup

a = Main.joystick_dev
binding.pry

