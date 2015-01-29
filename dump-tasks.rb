require 'orocos'


Orocos.initialize

#Orocos.default_pkgconfig_loader.available_deployments.each do |name, pkg|
#    STDOUT.puts "##{name} #{pkg}"
#end


class String
def norm()
    self.gsub('::','-').gsub('_','-')
end
end


def gen_stop_condition(tasks)

end

objects = Array.new
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
                tasks << task
                objects << "#{task.name}"
            end
        end
        rescue Exception => e
            STDOUT.puts "Error while loading #{project_name}"
        end
    end
    
    
    predicates << "\t (task ?x)"
    predicates << "\t (depends ?x ?y)"
    predicates << "\t (is-running ?x)"
    
    #file.puts "(:init"
    tasks.each do |t|
        initial << "\t (task #{t.name})"
    end
    
    
    goals << "(and "
    goals << "\t (and( (is-running t) (depends y t) ) )" #Running dependencies
    goals << ")"
    
#    actions <<  "(:action start :parameters (?r ?t)"
#    actions <<  " :effect (and    (is-running ?t) (depends ?r ?t) )"
#    actions <<  ")"
    
    actions <<  "(:action start :parameters (?r ?t)"
    actions <<  " :precondition (and (task ?t))"
    actions <<  " :effect (and    (is-running ?t) (depends ?r ?t) )"
    actions <<  ")"
   
    actions <<  "(:action stop :parameters (?t)"
    actions <<  " :precondition (not (depends _ ?t))"
    actions <<  " :effect (not    (is-running ?t))"
    actions <<  ")"


input_ports = Array.new
output_ports =Array.new

    Orocos.default_pkgconfig_loader.available_task_models.each do |name,project_name|
        begin
        if tasklib = Orocos.default_loader.project_model_from_name(project_name)
            if task = tasklib.self_tasks.find { |_, t| t.name == name }
                task = task.last
                task.output_ports.each do |p_name,p|
#                    objects << "#{p.task.name}.#{p.name}"
#                    output_ports << p
                end
                task.input_ports.each do |p_name,p|
#                    objects << "#{p.task.name}.#{p.name}"
#                    input_ports << p
                end
            end
        end
        rescue Exception => e
            STDOUT.puts "Error while loading #{project_name} #{e}"
        end
    end


    predicates <<  "\t (output-port ?x)"
    predicates <<  "\t (input-port ?x)"
    predicates <<  "\t (has-output ?x ?y)"
    predicates <<  "\t (has-input ?x ?y)"
    predicates <<  "\t (is-connected ?x ?y)"
    predicates <<  "\t (should-connected ?x ?y ?z)"
    
    output_ports.each do |p|
        initial << "\t (output-port #{p.task.name}.#{p.name})"
        initial << "\t (has-output #{p.task.name} #{p.task.name}.#{p.name})"
    end
    input_ports.each do |p|
    end

File.open('domain.pddl',File::CREAT|File::TRUNC|File::RDWR) do |file|
    file.puts "(define (domain network)"
    file.puts "  (:requirements :strips)"
    file.puts "  (:predicates"
    predicates.each {|p|  file.puts "#{p}"}
    file.puts ")"
    actions.each {|p|  file.puts p}
    file.puts ")"
end
File.open('problem.pddl',File::CREAT|File::TRUNC|File::RDWR) do |file|
    file.puts "(define (problem network001)"
    file.puts "  (:domain network)"
    file.puts "  (:objects"
    objects.each {|p|  file.puts "      #{p.norm}"}
    file.puts "  )"
    file.puts "  (:init"
    initial.each {|p|  file.puts "      #{p.norm}"}
    file.puts "  )"
    file.puts "  (:goal"
    goals.each {|p|  file.puts "      #{p.norm}"}
    file.puts "  )"
    file.puts ")"
end



    

#STDOUT.puts foo

