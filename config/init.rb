Roby.app.using 'syskit'


require 'roby/schedulers/temporal'
Roby.scheduler = Roby::Schedulers::Temporal.new


Syskit.conf.disable_logging
Syskit.conf.disable_conf_logging
Syskit.conf.transformer_enabled = true

