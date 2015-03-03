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

    def addDependancy(name, disabled=false)
        @depends << Requirement.new(name,disabled)
    end

    def metatype
        if task.instance_of? OroGen::Spec::TaskContext 
            "instance_req"
        elsif task.instance_of? OroGen::Spec::OutputPort
            "output_port"
        elsif task.instance_of? OroGen::Spec::InputPort
            "input_port"
        elsif task.kind_of? Syskit::Models::Composition
            "instance_req"
        elsif task.kind_of? Syskit::Models::DataServiceModel
            "instance_req"
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
        elsif task.kind_of? Syskit::Models::Composition
            "composition"
        elsif task.kind_of? Syskit::Models::DataServiceModel
            "data-service"
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
        if 
        task.instance_of? OroGen::Spec::TaskContext or 
        task.kind_of? Syskit::Models::DataServiceModel or
        task.kind_of? Syskit::Models::Composition

            task.name.gsub(' ','').escape
        elsif task.instance_of? OroGen::Spec::OutputPort or task.instance_of? OroGen::Spec::InputPort
            "#{task.task.name.to_s.gsub(' ','')}.#{task.name}".escape
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

#    a = Main.joystick_dev
#    binding.pry


    #Import Bundle definitions


    #Testing stuff

    #Test for start
    objects['root'].addDependancy("mars::IMU")

    #test for shutdown
    objects['mars::Joints'].running = true




binding.render("problem.pddl.template","problem.pddl")
