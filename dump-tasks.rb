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

class Requirement

    attr_writer :enabled
    attr_reader :name

    def disabled?
        !@enabled
    end

    def initialize(dep, enabled= true)
        @enabled = enabled
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

    def addDependancy(name, enabled=true)
        @requires<< Requirement.new(name,enabled)
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

    def initialize(task,type="")
        @depends = []
        @requires= []
        @task= task 
        @type = type
        @running = false
    end

    def is_running?
        @running
    end

    def addDependancy(name, disabled)
        @depends << Requirement.new(name,disabled)
    end

    def metatype
        if task.instance_of? OroGen::Spec::TaskContext 
            "instance_req"
        elsif task.instance_of? OroGen::Spec::OutputPort
            "output_port"
        elsif task.instance_of? OroGen::Spec::InputPort
            "input_port"
        else
            binding.pry
            "Unknown"
        end

    end

    def type
        if task.instance_of? OroGen::Spec::TaskContext 
            "task"
        elsif task.instance_of? OroGen::Spec::OutputPort
            nil
        elsif task.instance_of? OroGen::Spec::InputPort
            nil
        else
            binding.pry
            "Unknown"
        end
    end
    
    def input_ports
        
        if task.instance_of? OroGen::Spec::TaskContext
            a = Hash.new
            task.input_ports.each do |p|
                a[p[1].name] = p[1]
            end
            a.values
        else
            []
        end
    end
    
    def output_ports
        if task.instance_of? OroGen::Spec::TaskContext
            a = Hash.new
            task.output_ports.each do |p|
                a[p[1].name] = p[1]
            end
            a.values
        else
            [] 
        end
    end

    def name
        if task.instance_of? OroGen::Spec::TaskContext
            task.name
        elsif task.instance_of? OroGen::Spec::OutputPort or task.instance_of? OroGen::Spec::InputPort
            "#{task.task.name}.#{task.name}" 
        end
    end
end


Orocos.initialize

class Objects < Array

    def [] (op)
        if(op.instance_of? Fixnum)
            super[op]
        elsif (op.instance_of? String)
            self.each do |v|
                if v.name == op
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



    Orocos.default_pkgconfig_loader.available_task_models.each do |name,project_name|
        begin
        if tasklib = Orocos.default_loader.project_model_from_name(project_name)
            if task = tasklib.self_tasks.find { |_, t| t.name == name }
                task = task.last
                
                if(objects[task.name])
                    STDERR.puts "Warning Task #{task.name} already added" 
                    next
                end

                task.each_port do |port|
                    if(objects["#{port.task.name}.#{port.name}"])
                        STDERR.puts "Warning Port #{port.name} already added" 
                        next
                    end
                    objects <<  CObject.new(port)
                end
                objects <<  CObject.new(task)
            end
        end
        rescue Exception => e
            STDOUT.puts "Error while loading #{project_name}"
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
   
    a = Main.joystick_dev
    binding.pry


    #Import Bundle definitions


    #Testing stuff

    #Test for start
    objects['root'].addDependancy("mars::IMU")

    #test for shutdown
    objects['mars::Joints'].running = true




binding.render("problem.pddl.template","problem.pddl")
