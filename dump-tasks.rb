require 'orocos'
require 'erubis'

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

#Orocos.default_pkgconfig_loader.available_deployments.each do |name, pkg|
#    STDOUT.puts "##{name} #{pkg}"
#end


#class String
#def norm()
#    #self.gsub('::','-').gsub('_','-')
#    self.gsub('::','-')
#end
#end


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




    #Initial knot for startin tasks
    #objects << "root - instance_req" 
#    initial << "\t (task root)"
    #initial << "\t (is-running root)"
    #initial << "\t (depends root root)"
    objects << RootObject.new 

    Orocos.default_pkgconfig_loader.available_task_models.each do |name,project_name|
        begin
        if tasklib = Orocos.default_loader.project_model_from_name(project_name)
            if task = tasklib.self_tasks.find { |_, t| t.name == name }
    #            next if !project_name.include?("mars")  
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


  

    #Testing stuff
    objects['root'].addDependancy("mars::IMU")



#    goals << "
#    (forall (?t - instance_req)
#        (imply (is-running ?t)
#            (exists (?t2 - instance_req) (depends ?t2 ?t))
#        )
#    )
#" 
#
#    goals << "\t (is-running root)" #Running dependencies
#    goals << "\t (depends root mars-Camera)"
#    goals << "\t (depends root TestCmp)"
#
#    
#    actions <<  "(:action startt :parameters (?r - instance_req ?t - instance_req)"
#    actions <<  " :precondition (task ?t)"
#    actions <<  " :effect (and    (is-running ?t) (depends ?r ?t) )"
#    actions <<  ")"
#   
#    actions <<  "(:action stop :parameters (?t - instance_req)"
#    actions <<  " :precondition (not (depends _ ?t))"
#    actions <<  " :effect (not    (is-running ?t))"
#    actions <<  ")"
#
#    #Start composition
#    actions <<  
#"
#(:action startc :parameters (?r - instance_req ?c - instance_req)
# :precondition (and
#   (composition ?c)
#   (forall (?dep - instance_req) 
#    (imply (requires ?c ?dep) (is-running ?dep)) 
#   )
#  )
# :effect (and (is-running ?c) (depends ?r ?c) )
#)"
#
#
#input_ports = Array.new
#output_ports =Array.new
#
#    Orocos.default_pkgconfig_loader.available_task_models.each do |name,project_name|
#        begin
#        if tasklib = Orocos.default_loader.project_model_from_name(project_name)
#            if task = tasklib.self_tasks.find { |_, t| t.name == name }
#                task = task.last
#                next if !project_name.include?("mars")  
#                task.output_ports.each do |p_name,p|
##                    objects << "#{p.task.name}.#{p.name} - output_port"
##                    output_ports << p
#                end
#                task.input_ports.each do |p_name,p|
##                    objects << "#{p.task.name}.#{p.name} - input_port"
##                    input_ports << p
#                end
#            end
#        end
#        rescue Exception => e
#            STDOUT.puts "Error while loading #{project_name} #{e}"
#        end
#    end
#
#    #Test composition
#    objects << "TestCmp - instance_req"
#    initial << "(composition TestCmp)"
#    initial << "(requires TestCmp mars-IMU)"
#    initial << "(requires TestCmp mars-Task)"
#
#
#    predicates <<  "\t (has-output ?x - instance_req ?y - output_port)"
#    predicates <<  "\t (has-input ?x - instance_req ?y - input_port)"
#    predicates <<  "\t (is-connected ?x - output_port ?y - input_port)"
#    predicates <<  "\t (should-connected ?x - instance_req ?y - input_port ?z - output_port)"
#    predicates <<  "\t (requires ?x - instance_req ?y - instance_req)"
#    predicates <<  "\t (depends ?x - instance_req ?y - instance_req)"
#    predicates <<  "\t (is-running ?x - instance_req)"
#    predicates <<  "\t (task ?x - instance_req)"
#    predicates <<  "\t (composition ?x - instance_req)"
#    
#    output_ports.each do |p|
##        initial << "\t (output-port #{p.task.name}.#{p.name})"
#        initial << "\t (has-output #{p.task.name} #{p.task.name}.#{p.name})"
#    end
#    input_ports.each do |p|
#    end


#File.open('domain.pddl',File::CREAT|File::TRUNC|File::RDWR) do |file|
#    file.puts "(define (domain network)"
#    file.puts "  (:requirements :strips :equality :typing :conditional-effects)"
#    file.puts "  (:types input_port output_port instance_req)"
#    file.puts "  (:predicates"
#    predicates.each {|p|  file.puts "#{p}"}
#    file.puts ")"
#    actions.each {|p|  file.puts p}
#    file.puts ")"
#end


binding.render("problem.pddl.template","problem.pddl")

#File.open('problem.pddl',File::CREAT|File::TRUNC|File::RDWR) do |file|
#    file.puts "(define (problem network001)"
#    file.puts "  (:domain network)"
#    file.puts "  (:objects"
#    objects.each {|p|  file.puts "      #{p.norm}"}
#    file.puts "  )"
#    file.puts "  (:init"
#    initial.each {|p|  file.puts "      #{p.norm}"}
#    file.puts "  )"
#    file.puts "  (:goal (and"
#    goals.each {|p|  file.puts "      #{p.norm}"}
#    file.puts "  ))"
#    file.puts ")"
#end



    

#STDOUT.puts foo

