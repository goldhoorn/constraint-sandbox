require 'orocos'
require 'erubis'
require 'roby'
require 'syskit'
require './cobject'


class Binding
    def render(file, res)
        input = File.read(file)
        eruby = Erubis::Eruby.new(input)
        erg = eruby.result(self) 
        File.write(res,erg)
    end
end

class String
    def escape
        pattern = /(\'|\"|\*|\/|\-|\\|\)|\$|\+|\(|\^|\?|\!|\~|\`|\[|\]|<|>)/
        self.gsub(pattern){|match| "_"}
    end
end





Roby.app.robot 'avalon'
Roby.app.using_plugins 'syskit'
Syskit.conf.only_load_models = true
Syskit.conf.disables_local_process_server = true
Roby.app.single = true
Syskit.conf.disable_logging
Syskit.conf.disable_conf_logging

Roby.app.setup
#Orocos.initialize


    objects = CObjects.new

    objects << RootObject.new

    Syskit::TaskContext.each_submodel do |task|
        if(objects[task.name])
            STDERR.puts "Warning Task #{task.name} already added" 
            next
        end
        object = CObject.new(task)

        task.each_port do |port|
            if(objects["#{port.component_model.name}.#{port.name}"])
                STDERR.puts "Warning Port #{port.name} already added" 
                next
            end
            objects <<  CObject.new(port)
        end
        task.each_data_service do |name,service|
            service.each_fullfilled_model do |model|
                object.fullfills << model
            end
        end

        objects << object
    end

    Syskit::Composition.each_submodel do |cmp|
        o = CObject.new(cmp)
        cmp.each_child do |child_name,child|
            o.addDependancy(child.model.name.gsub(" ","").escape)
        end
        
        if(objects[o.name])
            STDERR.puts "Warning Cmp #{cmp.name} already added" 
            next
        end
        objects << o
    end
    
    Syskit::DataService.each_submodel do |ds|
        name = ds.name
        if(objects[name])
            STDERR.puts "Warning DataService #{name} already added" 
            next
        end
        objects <<  CObject.new(ds)
        ds.ports.each do |port|
            #TODO port typen handlen
        end
    end

    #Testing stuff

    #Test for start
    objects['root'].addDependancy("BatteryWatcher::Task")
    objects['root'].addDependancy("Base::SonarScanProviderSrv")
    #test for shutdown
    objects['DepthReader::Task'].running = true



    binding.render("problem.pddl.template","problem.pddl")
