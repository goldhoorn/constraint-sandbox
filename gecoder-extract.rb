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

def invalid(name)
    name.include?("Syskit::DataService") ||
    name.include?("Syskit::Composition") ||
    name.include?("Syskit::Component") ||
    name.include?("Roby::Task") ||
    name.include?("ControlLoop") ||
    name.include?("Syskit::ComBus") ||
    name.include?("Syskit::Device") ||
    name.include?("Base::ControlledSystemSrv") ||
    name.include?("Base::ControllerSrv") ||
    name.include?("Controller") ||
    name.include?("Controlled") ||
    false
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
        #Preevaluate if we have processed each child
        cmp.each_child do |child_name,child|
            next if invalid(child.model.name)
            service = nil
            service_name = child.model.fullfilled_model.first.name
            if (service = @data_services[service_name]).nil?
                if (service = @tasks[service_name]).nil?
                    if(service = @compositions[service_name]).nil?
                        STDERR.puts "Kacke hier #{child.model.name}"
                        @prunedCmps << cmp
                        return false
                    end
                end
            end
        end
       
        #Fine we can start adding
        #
        o = CObject.new(cmp)
        if(@objects[o.name])
            STDERR.puts "Warning Cmp #{cmp.name} already added  -- Debug this!!" 
            return true 
        end

        @compositions[cmp.name] = CONSTRAINED_BASED_NETWORKS::Composition.new(cmp.name)

        @dump.puts  "{"
        @dump.puts  "auto c = new Composition(\"#{cmp.name}\");"
        
        cmp.each_event do |symbol,object|
            @dump.puts "c->addEvent(\"#{symbol.to_s}\")"
        end

        cmp.each_child do |child_name,child|
            service = nil
            #Todo hacky assume first is the least abstract one
            service_name = child.model.fullfilled_model.first.name
            next if invalid(service_name)
          
#            if service_name.include?("DVL")
#                binding.pry
#            end

            #TODO handle optional childs instead of skipping them
            if child.optional?
                STDOUT.puts "Skipping optional child #{child.name} = #{service_name}"
                next
            end

            if (service = @data_services[service_name]).nil?
                if (service = @tasks[service_name]).nil?
                    if(service = @compositions[service_name]).nil?
                        raise "This is impossible"
                    end
                end
            end

                        
            @dump.puts "c->addChild(pool->getComponent(\"#{service_name}\"),\"#{child_name}\");" 
            @compositions[cmp.name].add_child(service, child_name)#child.model.name.gsub(" ","").escape)
#            o.addDependancy(child.model.name.gsub(" ","").escape)
        end
        
        cmp.each_fullfilled_model do |service|
            service.each_fullfilled_model do |model|
                next if invalid(model.name)
                @compositions[cmp.name].add_fullfillment model.name
                @dump.puts "c->addFullfillment(\"#{model.name}\");" 
                #o.fullfills << model
            end
        end
        @dump.puts "}"

        @objects << o
        return true
    end

#    binding.pry
#    exit 0
#    ::Main.each_action do |a|
#        if !a.coordination_model 
#            #We taking respect only to things thare a re repesent somehow a mission
#            next
#        end
#        a.coordination_model.each_task do |t|
#            name = t.name
#            model = t.model
#        end
#        #instance of start state might double
#        start_state = a.coordination_model.starting_state
#
#        a.coordination_model.transitions.each do |source,trigger,target|
#            source_state = source
#            trigger_object = trigger.task #.model ?
#            trigger_event = trigger.symbol
#            target_state = target
#        end
#
#    end

    begin
        dump = File.new("constraints.hpp",File::CREAT|File::TRUNC|File::RDWR, 0644)
        dump.puts "void load_constraints(){"
        dump.puts "using namespace constrained_based_networks;"
        dump.puts "auto pool = Pool::getInstance();"

        dep = Syskit::Actions::Profile.profiles.find{|p| p.name == "Avalon::Profiles::Avalon"}.dependency_injection.each.to_a
        i=0
        while i+1 < dep.size 
            source = dep[i]
            target = dep[i+1]
          #  binding.pry
            STDOUT.puts "Size: #{dep.size} #{i}" 
            name = source.name
            keys = target.instance_eval("selections").each_selection_key.to_a
            values = target.instance_eval("selections").each.to_a
            raise "Arrays does not match their size" if keys.size != values.size
            for j in 0..(keys.size-1)
                #TODO filter out sub-model specifications here
                target = ""
                if values[j].respond_to?(:component_model)
                    target = values[j].component_model.name
                end
                if target == ""
                    target = values[j].name
                end
                dump.puts "\t{"
                dump.puts "\tauto c = pool->getComponent(\"" + name + "\");"
                dump.puts "\tif(auto cmp = dynamic_cast<Composition*>(c)){"
                dump.puts "\t\tcmp->addConstraint(\"" + keys[j] + "\",\"" + target + "\");"
                dump.puts "\t}else{ std::cerr << \"FATAL cannot cast to composition\" << std::endl;}"
                dump.puts "\t}"
                #binding.pry
                STDOUT.puts "#{name}.#{keys[j]} == #{target}"
            end
            i=i+2
        end
        dump.puts "}"
    end

#    Syskit::Actions::Profile.profiles.find{|p| p.name == "Avalon::Profiles::Avalon"}.definitions.each do |name,definition|
#        #TODO hacky
#        keys = definition.instance_eval("pushed_selections").each_selection_key.to_a
#        values = definition.instance_eval("pushed_selections").each.to_a
#        raise "Arrays does not match their size" if keys.size != values.size
#        for i in 0..(keys.size-1)
#            STDOUT.puts "#{name}.#{keys[i]} == #{values[i]}"
#        end
#    end
#    binding.pry

    dump = File.new("dump.hpp",File::CREAT|File::TRUNC|File::RDWR, 0644)
    dump.puts "#include \"constraints.hpp\";"
    dump.puts "std::string create_model(){"
    dump.puts "using namespace constrained_based_networks;"
    dump.puts "auto pool = Pool::getInstance();"
    
    @dump = dump

    Syskit::DataService.each_submodel do |ds|
        next if invalid(ds.name)

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
        task.each_event do |symbol,object|
            dump.puts "t->addEvent(\"#{symbol.to_s}\")"
        end

#        task.each_port do |port|
#            if(objects["#{port.component_model.name}.#{port.name}"])
#                STDERR.puts "Warning Port #{port.name} already added" 
#                next
#            end
#            objects <<  CObject.new(port)
#        end
        task.each_data_service do |name,service|
            service.each_fullfilled_model do |model|
                next if invalid(model.name)
                tasks[task.name].add_fullfillment model.name
                dump.puts  "t->addFullfillment(\"#{model.name}\");" 
            end
        end
        dump.puts "}"
        objects << object
    end

    Syskit::Composition.each_submodel do |cmp|
        next if invalid(cmp.name)
        handleComp(cmp)
    end

    size_changed = true
    while(prunedCmps.size != 0 && size_changed) do
        prunedCmps2 = prunedCmps.dup
        prunedCmps2.each do |cmp|
            next if invalid(cmp.name)
            if handleComp(cmp)
                prunedCmps.delete(cmp)
            end
        end
        size_changed = prunedCmps2.size != prunedCmps.size 
        STDOUT.puts "Size old: #{prunedCmps2.size} new: #{prunedCmps.size}"
    end

    dump.puts "load_constraints();" 
    dump.puts "return \"\";" 
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

#    compositions.each do |name,c|
#        c.unsolveable_childs.each do |un|
#            STDERR.puts "#{un} is Unresolveable"
#        end
#    end





