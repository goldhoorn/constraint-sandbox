require 'orocos'
require 'nokogiri'
#require 'erubis'
require 'roby'
require 'syskit'
#require 'constrained_based_networks'

require_relative 'cobject'
FILENAME="avalon_model.xml"

@xml = nil

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
        #    name.include?("ControlLoop") ||
        name.include?("Syskit::ComBus") ||
        name.include?("Syskit::Device") ||
        #    name.include?("Base::ControlledSystemSrv") ||
        #    name.include?("Base::ControllerSrv") ||
        #    name.include?("Controller") ||
        #    name.include?("Controlled") ||
        #    name.include?("is_a") ||
        false
end

def base_name(name)
    name.to_s.split("/").first
end

def generate_specialized_if_needed(name, ir, id)
    model = ir.model
    action = ir.action

    if action.arguments.each.to_a.any?{|k,v| v} 
        @xml.child(:name =>  base_name(model.to_s), :specialized => true, :id => id){
            action.arguments.each do |k,v|
                @xml.config(:name => k.to_s, :value => v.to_s)
            end
        }
    else
        @xml.child(name: base_name(model.to_s), specialized: false, :id=> id )
    end
    id = id+1
    #    extra_args.each do |a|
    #        dump.puts a
    #    end
    id-1
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

    @compositions[cmp.name] = cmp.name # CONSTRAINED_BASED_NETWORKS::Composition.new(cmp.name)

    if cmp.name != base_name(cmp.name)
        #cmp got already added as a more-abstract one
        return true
    end

    @xml.composition(name: base_name(cmp.name)){

        cmp.each_event do |symbol,object|
            @xml.event(name: symbol.to_s)
        end

        cmp.arguments.each do |arg|
            type = cmp.argument_types[arg]
            if type == :double
                @xml.property(name: arg.to_s, type: "ConfigurationModel::DOUBLE")
            elsif type == :string
                @xml.property(name: arg.to_s, type: "ConfigurationModel::STRING")
            elsif type == :bool
                @xml.property(name: arg.to_s, type: "ConfigurationModel::BOOL")
            elsif type == :int
                @xml.property(name: arg.to_s, type: "ConfigurationModel::INT")
            elsif type == :config
                #TODO handle configs *WARGH*
            elsif type == :ignore
                #TODO nothing
            elsif type.nil?
                STDERR.puts "WTF nil type for #{arg} of #{cmp.name}"
            else
                raise ArgumentError, "Unknown type #{type} for #{arg} of #{cmp.name}"
            end
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

            @xml.child(name: base_name(service_name), role: child_name)
        end

        if(cmp.respond_to?(:argument_forwards))
            cmp.argument_forwards.each do |child, events|
                events.each do |source,target|
                    @xml.argument_forward(child: child.to_s, source_argument: source.to_s, target_argument: target.to_s)
                end
            end
        end

        if(cmp.respond_to?(:emits))
            cmp.emits.each do |e|
                @xml.emits(name: e.to_s)
            end
        end

        if(cmp.respond_to?(:event_forwards))
            cmp.event_forwards.each do |child, events|
                events.each do |source,target|
                    @xml.event_forward(child: child.to_s, source_event: source.to_s, target_event: target.to_s)
                end
            end
        end

        cmp.each_fullfilled_model do |service|
            service.each_fullfilled_model do |model|
                next if invalid(model.name)
                #@compositions[cmp.name].add_fullfillment model.name
                @xml.fullfills(service: base_name(model.name))
            end
        end

        @objects << o
    }
    return true
end

builder = Nokogiri::XML::Builder.new do |xml|
    @xml=xml
    @xml.root{
        @xml.initial_model(file: FILENAME)

        #dump = File.new("state_machines.hpp",File::CREAT|File::TRUNC|File::RDWR, 0644)
        #dump.puts "void load_state_machines(constrained_based_networks::Pool *pool){"
        #dump.puts "using namespace constrained_based_networks;"
        ::Main.each_action do |a|
            #TODO hier weiter
            if !a.coordination_model 
                #We taking respect only to things thare a re repesent somehow a mission
                next
            end
            if !a.coordination_model.respond_to?("starting_state")
                #Only catching the name got a unsupported action_script
                @xml.state_machine(name: a.returned_type.to_s)
                next
            end


            @xml.state_machine(name: a.returned_type.to_s, start_state: 0){

                start_state = a.coordination_model.starting_state

                id = 0
                st = generate_specialized_if_needed("start_state",start_state, id)
                id = st+1 
                a.coordination_model.transitions.each do |source,trigger,target|
                    source_state = source
                    trigger_object = trigger.task #.model ?
                    trigger_event = trigger.symbol
                    target_state = target

                    #binding.pry if a.name.include?("blind_forward_and_back")
                    #TODO build real models of states
                    so = generate_specialized_if_needed("source",source,id)
                    id = so+1 
                    ta = generate_specialized_if_needed("target",target,id)
                    id = ta+1 
                    tr = generate_specialized_if_needed("trigger",trigger_object,id)
                    id = tr+1 
                    @xml.transition(source: so, target: ta, trigger: tr, event: trigger_event.to_s)

                end
            }
        end

        dep = Syskit::Actions::Profile.profiles.find{|p| p.name == "Avalon::Profiles::Avalon"}.dependency_injection.each.to_a
        i=0
        while i+1 < dep.size 
            source = dep[i]
            target = dep[i+1]
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
                @xml.constraint(component: base_name( name), child: keys[j], target: target)
            end
            i=i+2
        end

        Syskit::DataService.each_submodel do |ds|
            next if invalid(ds.name)

            name = ds.name

            if(objects[name])
                STDERR.puts "Warning DataService #{name} already added" 
                next
            end

            data_services[name] = name #CONSTRAINED_BASED_NETWORKS::DataService.new(name)
            @xml.data_service(name: name)

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

            tasks[task.name] = task.name #CONSTRAINED_BASED_NETWORKS::Task.new(task.name)
            @xml.task(name: task.name){

                task.each_event do |symbol,object|
                    @xml.event(name: symbol.to_s)
                end

                if c = Orocos.conf.conf[task.orogen_model.name]
                    c.sections.each do |n,s|
                        s.each do |k,v|
                            @xml.config_file(section: n.to_s, property: k.to_s, value: v.to_s)
                        end
                    end
                end

                task.each_data_service do |name,service|
                    service.each_fullfilled_model do |model|
                        next if invalid(model.name)
                        @xml.fullfills(service: base_name(model.name))
                    end
                end

                task.orogen_model.each_property do |prop|
                    type = nil 
                    begin
                        type = prop.type.convertion_to_ruby[0]
                    rescue Exception => e
                        STDERR.puts "Cannot handle #{prop} with type #{prop.type} of #{task.name}"
                        next
                    end

                    if type <= Float
                        @xml.property(name: prop.name.to_s, type: "ConfigurationModel::DOUBLE")
                    elsif type <= String 
                        @xml.property(name: prop.name.to_s, type: "ConfigurationModel::STRING")
                    elsif prop.type.name == "/bool" 
                        @xml.property(name: prop.name.to_s, type: "ConfigurationModel::BOOL")
                    elsif type <= Integer || prop.type.name <= "/int32_t"
                        @xml.property(name: prop.name.to_s, type: "ConfigurationModel::INT")
                        #            elsif type == "/config"
                        #                #TODO handle configs *WARGH*
                        #            elsif type == :ignore
                        #                #TODO nothing
                    elsif type.nil?
                        STDERR.puts "WTF nil type for #{prop} of #{task.name}"
                    else
                        STDERR.puts "Cannot handle #{prop} with type #{prop.type} of #{task.name}"
                    end
                end
                objects << object
            }
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
    }
end


file = File.new(FILENAME,File::CREAT|File::TRUNC|File::RDWR, 0644)
file.write builder.to_xml
file.close
STDOUT.puts "Ende"
exit 0





