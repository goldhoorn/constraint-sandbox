require 'orocos'
require 'erubis'
require 'roby'
require 'syskit'
require 'constrained_based_networks'

require_relative 'cobject'


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
   
    data_services = Hash.new
    tasks = Hash.new
    compositions = Hash.new
    
    Syskit::DataService.each_submodel do |ds|
        name = ds.name
#        STDERR.puts "DS: #{name}"

        if(objects[name])
            STDERR.puts "Warning DataService #{name} already added" 
            next
        end

#        constrained_based_networks::Task 

        data_services[name] = CONSTRAINED_BASED_NETWORKS::DataService.new(name)
        
        STDOUT.puts "Adding DataService #{name}"

        objects <<  CObject.new(ds)
        ds.ports.each do |port|
            #TODO port typen handlen
        end
    end

    Syskit::TaskContext.each_submodel do |task|
        if(objects[task.name])
            STDERR.puts "Warning Task #{task.name} already added" 
            next
        end
        object = CObject.new(task)
        
        tasks[task.name] = CONSTRAINED_BASED_NETWORKS::Task.new(task.name)

#        task.each_port do |port|
#            if(objects["#{port.component_model.name}.#{port.name}"])
#                STDERR.puts "Warning Port #{port.name} already added" 
#                next
#            end
#            objects <<  CObject.new(port)
#        end
        task.each_data_service do |name,service|
            service.each_fullfilled_model do |model|
                tasks[task.name].add_fullfillment model.name
#                object.fullfills << model
            end
        end

        objects << object
    end

    Syskit::Composition.each_submodel do |cmp|

        o = CObject.new(cmp)
        if(objects[o.name])
            STDERR.puts "Warning Cmp #{cmp.name} already added" 
            next
        end

        compositions[cmp.name] = CONSTRAINED_BASED_NETWORKS::Composition.new(cmp.name)
        
        cmp.each_child do |child_name,child|
            service = nil
            if (service = data_services[child.model.name]).nil?
                if (service = tasks[child.model.name]).nil?
                    if(service = compositions[child.model.name]).nil?
                        data_services[child.model.name] = CONSTRAINED_BASED_NETWORKS::DataService.new(child.model.name)
#                        STDERR.puts "Kacke hier #{child.model.name}"
#                        next
                        #This is a workaround because everything is a DS
                        service = data_services[child.model.name]
                    end
                end
            end
#            STDERR.puts "Alles gut #{child.model.name}"

            compositions[cmp.name].add_child( service, child_name)#child.model.name.gsub(" ","").escape)
#            o.addDependancy(child.model.name.gsub(" ","").escape)
        end
        
        cmp.each_fullfilled_model do |service|
            service.each_fullfilled_model do |model|
                compositions[cmp.name].add_fullfillment model.name
                #o.fullfills << model
            end
        end

        objects << o
    end

    #compositions['AuvControl::DepthFusionCmp'].active(true)
    #compositions['Pipeline::Follower'].active(true)
#    compositions['Pipeline::Detector_new'].active(true)
    #compositions['Localization::ParticleDetector'].active(true)
    #compositions['Localization::ParticleDetector'].active(true)
    #compositions['Buoy::DetectorNewCmp'].active(true)
    #compositions['Pipeline::Follower/[controller.is_a?(Base::AUVRelativeMotionControllerSrv), controlled_system.is_a?(Base::AUVRelativeMotionControlledSystemSrv)]'].active(true)
#    compositions['Base::ControlLoop/[controller.is_a?(AvalonControl::MotionControlTask)]'].active(true) #ok
    compositions['Base::ControlLoop/[controller.is_a?(Base::JointsControlledSystemSrv)]'].active(true) #
    
    file = File.new("Compositions.txt",File::CREAT|File::TRUNC|File::RDWR, 0644)
    compositions.each do |c,v|
        file.puts c
        c.each_child do |child_name,child|
            file.puts "-- #{child.model.name}"
        end
    end
    file.close

    #Testing stuff

    #Test for start
#    objects['root'].addDependancy("BatteryWatcher::Task")
#    objects['root'].addDependancy("Base::SonarScanProviderSrv",false)
#    objects['root'].addDependancy("Pipeline::Follower")


#    objects['root'].addDependancy("PoseAuv::IKFOrientationEstimatorCmp",false)
#    objects['root'].addDependancy("BatteryWatcher::Task",false)
#    objects['root'].addDependancy("Base::SonarScanProviderSrv",false)
    #test for shutdown
#    objects['DepthReader::Task'].running = true

    s = CONSTRAINED_BASED_NETWORKS::Solution.bab_search
    #s = CONSTRAINED_BASED_NETWORKS::Solution.gist_search
    s.print



#if(File.exists?("problem.pddl.template"))
#    binding.render("problem.pddl.template","problem.pddl")
#end
#
#if(File.exists?("problem.template"))
#    binding.render("problem.template","problem")
#end


