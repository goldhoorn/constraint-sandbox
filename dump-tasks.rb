require 'orocos'
require 'erubis'
require 'roby'
require 'syskit'



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
class Requirement

    attr_writer :enabled
    attr_reader :name

    def disabled?
        @disabled
    end

    def initialize(dep, disabled= false)
        @disabled=disabled 
        @name = dep
    end
end


class RootObject
    attr_accessor :requires

    def initialize
        @requires = []
        @requires << Requirement.new("root")
    end
   
    def is_running?
        true
    end

    def addDependancy(name, disabled=false)
        @requires<< Requirement.new(name,disabled)
    end

    def type
        "task"
    end
    
    def metatype 
        "instance_req"
    end

    def name
        "root"
    end
   
    def input_ports
        []
    end
    
    def output_ports
        []
    end

    def each_fullfillment
        [].each
    end

    def depends 
        [Requirement.new("root")]
        #["root"]
    end

end

class CObject
    attr_accessor :task
    attr_accessor :type
    attr_accessor :depends
    attr_accessor :requires
    attr_writer :running
    attr_accessor :fullfills

    def initialize(task,type="")
        @depends = []
        @requires= []
        @task= task 
        @type = type
        @running = false
        @fullfills = []
    end

    def is_running?
        @running
    end

    def addDependancy(name, disabled=false)
        @depends << Requirement.new(name,disabled)
    end

    def each_fullfillment
        return enum_for(:each_fullfillment) unless block_given?
        fullfills.each do |value|
            yield(value.name.escape)
        end
    end

    def metatype
        if task.kind_of? Syskit::Models::OutputPort 
#        elsif task.instance_of? OroGen::Spec::OutputPort
            "output_port"
        elsif task.kind_of? Syskit::Models::InputPort 
        #elsif task.instance_of? OroGen::Spec::InputPort
            "input_port"
        elsif task.kind_of? Syskit::Models::Composition
            "instance_req"
        elsif task.kind_of? Syskit::Models::DataServiceModel
            "instance_req"
        elsif task < Syskit::TaskContext
        #if task.instance_of? OroGen::Spec::TaskContext 
            "instance_req"
        else
            binding.pry
            "Unknown"
        end

    end

    def type
        if task.kind_of? Syskit::Models::OutputPort 
        #elsif task.instance_of? OroGen::Spec::OutputPort
            nil
        elsif task.kind_of? Syskit::Models::InputPort 
        #elsif task.instance_of? OroGen::Spec::InputPort
            nil
        elsif task.kind_of? Syskit::Models::Composition
            "composition"
        elsif task.kind_of? Syskit::Models::DataServiceModel
            "data-service"
        elsif task < Syskit::TaskContext
        #if task.instance_of? OroGen::Spec::TaskContext 
            "task"
        else
            binding.pry
            "Unknown"
        end
    end
    
    def input_ports
        begin
            if task < Syskit::TaskContext
            #if task.instance_of? OroGen::Spec::TaskContext
                a = Hash.new
                task.input_ports.each do |p|
                    a[p[1].name] = p[1]
                end
                return a.values
            end
        rescue Exception => e
            ##Nothing
        end
        
        return []
    end
    
    def output_ports
        begin
            if task < Syskit::TaskContext
    #        if task.instance_of? OroGen::Spec::TaskContext
                a = Hash.new
                task.output_ports.each do |p|
                    a[p[1].name] = p[1]
                end
                return a.values
            end
        rescue Exception => e
            #nothing
        end
        return [] 
    end

    def name
        if 
#        task.instance_of? OroGen::Spec::TaskContext or 
        task.kind_of? Syskit::Models::DataServiceModel or 
        task.kind_of? Syskit::Models::Composition 
            task.name.gsub(' ','').escape
        elsif 
#            task.instance_of? OroGen::Spec::OutputPort
#            task.instance_of? OroGen::Spec::InputPort
            task.instance_of? Syskit::Models::InputPort or 
            task.instance_of? Syskit::Models::OutputPort
            "#{task.component_model.name.to_s.gsub(' ','')}.#{task.name}".escape
        elsif
        task < Syskit::TaskContext 
            task.name.gsub(' ','').escape
        else
            binding.pry
        end
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

class Objects < Array

    def [] (op)
        if(op.instance_of? Fixnum)
            super[op]
        elsif (op.instance_of? String)
            self.each do |v|
                if v.name == op.escape
                    return v
                end
            end
        end
        nil
    end
end

objects = Objects.new
predicates = Array.new
initial = Array.new
goals = Array.new
actions = Array.new

tasks = Array.new

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
#            name = child[0]
#            binding.pry
#            child = child[1] # composition child
#            objects << CObject.new(child)
            #o.addDependancy(name.escape)
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
    #test for shutdown
    objects['DepthReader::Task'].running = true

#    STDOUT.puts "Size: #{objects['DepthReader::Task'].each_fullfillment.to_a.size}"
#    objects['DepthReader::Task'].each_fullfillment do |v|
#        binding.pry
#    end




binding.render("problem.pddl.template","problem.pddl")
