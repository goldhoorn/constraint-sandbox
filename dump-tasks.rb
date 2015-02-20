require 'orocos'


Orocos.initialize

#Orocos.default_pkgconfig_loader.available_deployments.each do |name, pkg|
#    STDOUT.puts "##{name} #{pkg}"
#end


class String
def norm()
    #self.gsub('::','-').gsub('_','-')
    self.gsub('::','-')
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




    #Initial knot for startin tasks
    objects << "root - instance_req" 
#    initial << "\t (task root)"
    initial << "\t (is-running root)"
    initial << "\t (depends root root)"


    Orocos.default_pkgconfig_loader.available_task_models.each do |name,project_name|
        begin
        if tasklib = Orocos.default_loader.project_model_from_name(project_name)
            if task = tasklib.self_tasks.find { |_, t| t.name == name }
                next if !project_name.include?("mars")  
                task = task.last
                tasks << task
                objects << "#{task.name} - instance_req"
                initial << "\t (task #{task.name})"
            end
        end
        rescue Exception => e
            STDOUT.puts "Error while loading #{project_name}"
        end
    end
   
    goals << "
    (forall (?t - instance_req)
        (imply (is-running ?t)
            (exists (?t2 - instance_req) (depends ?t2 ?t))
        )
    )
" 

    goals << "\t (is-running root)" #Running dependencies
    goals << "\t (depends root mars-Camera)"
    goals << "\t (depends root TestCmp)"

    
    actions <<  "(:action startt :parameters (?r - instance_req ?t - instance_req)"
    actions <<  " :precondition (task ?t)"
    actions <<  " :effect (and    (is-running ?t) (depends ?r ?t) )"
    actions <<  ")"
   
    actions <<  "(:action stop :parameters (?t - instance_req)"
    actions <<  " :precondition (not (depends _ ?t))"
    actions <<  " :effect (not    (is-running ?t))"
    actions <<  ")"

    #Start composition
    actions <<  
"
(:action startc :parameters (?r - instance_req ?c - instance_req)
 :precondition (and
   (composition ?c)
   (forall (?dep - instance_req) 
    (imply (requires ?c ?dep) (is-running ?dep)) 
   )
  )
 :effect (and (is-running ?c) (depends ?r ?c) )
)"


input_ports = Array.new
output_ports =Array.new

    Orocos.default_pkgconfig_loader.available_task_models.each do |name,project_name|
        begin
        if tasklib = Orocos.default_loader.project_model_from_name(project_name)
            if task = tasklib.self_tasks.find { |_, t| t.name == name }
                task = task.last
                next if !project_name.include?("mars")  
                task.output_ports.each do |p_name,p|
#                    objects << "#{p.task.name}.#{p.name} - output_port"
#                    output_ports << p
                end
                task.input_ports.each do |p_name,p|
#                    objects << "#{p.task.name}.#{p.name} - input_port"
#                    input_ports << p
                end
            end
        end
        rescue Exception => e
            STDOUT.puts "Error while loading #{project_name} #{e}"
        end
    end

    #Test composition
    objects << "TestCmp - instance_req"
    initial << "(composition TestCmp)"
    initial << "(requires TestCmp mars-IMU)"
    initial << "(requires TestCmp mars-Task)"


    predicates <<  "\t (has-output ?x - instance_req ?y - output_port)"
    predicates <<  "\t (has-input ?x - instance_req ?y - input_port)"
    predicates <<  "\t (is-connected ?x - output_port ?y - input_port)"
    predicates <<  "\t (should-connected ?x - instance_req ?y - input_port ?z - output_port)"
    predicates <<  "\t (requires ?x - instance_req ?y - instance_req)"
    predicates <<  "\t (depends ?x - instance_req ?y - instance_req)"
    predicates <<  "\t (is-running ?x - instance_req)"
    predicates <<  "\t (task ?x - instance_req)"
    predicates <<  "\t (composition ?x - instance_req)"
    
    output_ports.each do |p|
#        initial << "\t (output-port #{p.task.name}.#{p.name})"
        initial << "\t (has-output #{p.task.name} #{p.task.name}.#{p.name})"
    end
    input_ports.each do |p|
    end


File.open('domain.pddl',File::CREAT|File::TRUNC|File::RDWR) do |file|
    file.puts "(define (domain network)"
    file.puts "  (:requirements :strips :equality :typing :conditional-effects)"
    file.puts "  (:types input_port output_port instance_req)"
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
    file.puts "  (:goal (and"
    goals.each {|p|  file.puts "      #{p.norm}"}
    file.puts "  ))"
    file.puts ")"
end



    

#STDOUT.puts foo

