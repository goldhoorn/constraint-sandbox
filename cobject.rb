

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


class BaseObject
    attr_accessor :requires
    attr_accessor :depends
    attr_accessor :fullfills
    attr_writer :running
    
    def initialize
        @requires = []
        @depends = []
        @fullfills = []
        @running = false
    end
    
    def valid_model?(model)
        if
            model.name == "Roby::Task" or
            model.name == "Syskit::Composition" or
            model.name == "Syskit::Component" or 
            model.name == "Syskit::DataService"
            return false 
        else
#            binding.pry
#            STDOUT.puts "Class: #{model.class.name}"
            return true
        end
    end
    
    def input_ports
        []
    end
    
    def output_ports
        []
    end
    
    def is_running?
        @running
    end
    
    def addDependancy(name, disabled=false)
        @requires<< Requirement.new(name,disabled)
    end
    
    def each_fullfillment
        return enum_for(:each_fullfillment) unless block_given?
        fullfills.each do |value|
            if valid_model?(value)
                yield(value.name.escape)
            end
        end
    end

end

class RootObject < BaseObject

    def initialize
        super
#        @requires << Requirement.new("root")
#        @depends = [Requirement.new("root")]
        @running = false 
    end
   
    def type
        "composition"
    end
    
    def metatype 
        "instance_req"
    end

    def name
        "root"
    end
end

class CObject < BaseObject
    attr_accessor :task
#    attr_accessor :type

    #def initialize(task,type="")
    def initialize(task)
        super()
        @task= task 
    end


    def addDependancy(name, disabled=false)
        @depends << Requirement.new(name,disabled)
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

class CObjects < Array

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
