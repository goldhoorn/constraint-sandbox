require 'orocos'
#require 'erubis'
require 'roby'
require 'syskit'
require 'constrained_based_networks'

require_relative 'cobject'


#class Binding
#    def render(file, res)
#        input = File.read(file)
#        eruby = Erubis::Eruby.new(input)
#        erg = eruby.result(self) 
#        File.write(res,erg)
#    end
#end
#




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
    objects  << RootObject.new
    data_services = Hash.new
    tasks = Hash.new
    compositions = Hash.new
    prunedCmps = Array.new
    @objects = objects
    @data_services = data_services
    @tasks = tasks
    @compositions = compositions
    @prunedCmps = prunedCmps
    
    def handleComp(cmp)
        o = nil

        cmp.each_child do |child_name,child|
            service = nil
            #Todo hacky assume first is the least abstract one
            service_name = child.model.fullfilled_model.first.name

            if (service = @data_services[service_name]).nil?
                if (service = @tasks[service_name]).nil?
                    if(service = @compositions[service_name]).nil?
                        #This is a workaround because everything is a DS
                        #Hack assuing we neet to create one
                        STDERR.puts "Kacke hier #{child.model.name}"
                        @prunedCmps << cmp
                        return false
                        #compositions[service_name] = CONSTRAINED_BASED_NETWORKS::Composition.new(service_name)
                        #service = compositions[service_name]
                        #dump.puts "new DataService(\"#{service_name}\");" 
                    end
                end
            end

            o = CObject.new(cmp)
            if(@objects[o.name])
                STDERR.puts "Warning Cmp #{cmp.name} already added  -- Debug this!!" 
                return true 
            end

            @compositions[cmp.name] = CONSTRAINED_BASED_NETWORKS::Composition.new(cmp.name)

            @dump.puts  "{"
            @dump.puts  "auto c = new Composition(\"#{cmp.name}\");"
                        
            @dump.puts "c->addChild(pool->getComponent(\"#{service_name}\"),\"#{child_name}\");" 
            @compositions[cmp.name].add_child(service, child_name)#child.model.name.gsub(" ","").escape)
#            o.addDependancy(child.model.name.gsub(" ","").escape)
        end
        
        cmp.each_fullfilled_model do |service|
            service.each_fullfilled_model do |model|
                @compositions[cmp.name].add_fullfillment model.name
                @dump.puts "c->addFullfillment(\"#{model.name}\");" 
                #o.fullfills << model
            end
        end
        @dump.puts "}"

        @objects << o
        return true
    end

    dump = File.new("dump.hpp",File::CREAT|File::TRUNC|File::RDWR, 0644)
    dump.puts "void create_model(){"
    dump.puts "using namespace constrained_based_networks;"
    dump.puts "auto pool = Pool::getInstance();"
    
    @dump = dump

    Syskit::DataService.each_submodel do |ds|
        name = ds.name
#        STDERR.puts "DS: #{name}"

        if(objects[name])
            STDERR.puts "Warning DataService #{name} already added" 
            next
        end

#        constrained_based_networks::Task 

        data_services[name] = CONSTRAINED_BASED_NETWORKS::DataService.new(name)
        dump.puts "new DataService(\"#{name}\");" 

        #STDOUT.puts "Adding DataService #{name}"

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
        dump.puts  "{"
        dump.puts  "auto t = new Task(\"#{task.name}\");" 
        dump.puts  "(void)t;"

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
                dump.puts  "t->addFullfillment(\"#{model.name}\");" 
            end
        end
        dump.puts "}"
        objects << object
    end

    Syskit::Composition.each_submodel do |cmp|
        handleComp(cmp)
    end

    size_changed = true
    while(prunedCmps.size != 0 && size_changed) do
        prunedCmps2 = prunedCmps.dup
        prunedCmps2.each do |cmp|
            if handleComp(cmp)
                prunedCmps.delete(cmp)
            end
        end
        size_changed = prunedCmps2.size != prunedCmps.size 
        STDOUT.puts "Size old: #{prunedCmps2.size} new: #{prunedCmps.size}"
    end
    
    dump.puts "}"

    STDOUT.puts ""
    STDOUT.puts "#######################################################################################"
    STDOUT.puts ""
    if prunedCmps.size != 0
        STDERR.puts "#{prunedCmps.size} compositions are still unresolveable"
        prunedCmps.each do |cmp|
            STDOUT.puts "-- #{cmp.name}"
        end
        exit -1
    end
    #compositions['AuvControl::DepthFusionCmp'].active(true)
#    compositions['Pipeline::Follower/[controller.is_a?(Base::AUVRelativeMotionControllerSrv), controlled_system.is_a?(Base::AUVRelativeMotionControlledSystemSrv)]'].active(true)
#    compositions['Pipeline::Follower'].active(true)
    #compositions['Pipeline::Follower/[controller.is_a?(Base::AUVRelativeMotionControllerSrv), controlled_system.is_a?(Base::AUVRelativeMotionControlledSystemSrv)]'].active(true)
    #compositions['Pipeline::Follower/[controller.is_a?(Base::AUVRelativeMotionControllerSrv), controlled_system.is_a?(Base::AUVRelativeMotionControlledSystemSrv)]'].active(true)
#    compositions['Pipeline::Detector_new'].active(true)
    #compositions['Localization::ParticleDetector'].active(true)
    #compositions['Localization::ParticleDetector'].active(true)
    #compositions['Buoy::DetectorNewCmp'].active(true)
    #compositions['Pipeline::Follower/[controller.is_a?(Base::AUVRelativeMotionControllerSrv), controlled_system.is_a?(Base::AUVRelativeMotionControlledSystemSrv)]'].active(true)
#    compositions['Base::ControlLoop/[controller.is_a?(AvalonControl::MotionControlTask)]'].active(true) #ok
    #compositions['Base::ControlLoop/[controller.is_a?(Base::JointsControlledSystemSrv)]'].active(true) #
    #compositions['Hbridge::ControlSystem'].active(true) #ok
    #compositions['Base::ControlLoop/[controller.is_a?(Base::AUVRelativeMotionControlledSystemSrv)]'].active(true) 
    #compositions['Pipeline::Detector'].active(true) 
    
    file = File.new("Compositions.txt",File::CREAT|File::TRUNC|File::RDWR, 0644)
    Syskit::Composition.each_submodel do |cmp|
        file.puts "-C- #{cmp.name}"
        cmp.each_child do |child_name,child|
            file.puts "-Ci- #{child.model.name}"
        end
        cmp.each_fullfilled_model do |service|
            service.each_fullfilled_model do |model|
                file.puts "-F--- #{model.name}"
            end
        end
    end
    Syskit::TaskContext.each_submodel do |task|
        file.puts "-T- #{task.name}"
        task.each_fullfilled_model do |service|
            service.each_fullfilled_model do |model|
                file.puts "-F--- #{model.name}"
            end
        end
    end
    file.close

    compositions.each do |name,c|
        c.unsolveable_childs.each do |un|
            STDERR.puts "#{un} is Unresolveable"
        end
    end

#    STDOUT.puts "Beginning solution calculation"
#    p = CONSTRAINED_BASED_NETWORKS::Pool.instance
#    p.save("dataset.dump")

    begin
        #s = CONSTRAINED_BASED_NETWORKS::Solution.gist_search
    #    s = CONSTRAINED_BASED_NETWORKS::Solution.bab_search
     #   s.print
    rescue Exception => e
        STDERR.puts "Exception #{e} occured"
    end



#if(File.exists?("problem.pddl.template"))
#    binding.render("problem.pddl.template","problem.pddl")
#end
#
#if(File.exists?("problem.template"))
#    binding.render("problem.template","problem")
#end


