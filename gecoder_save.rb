require 'constrained_based_networks'


   
data_services = Hash.new
tasks = Hash.new
compositions = Hash.new

name = "Base::OrientationSrv"
data_services[name] = CONSTRAINED_BASED_NETWORKS::DataService.new(name)

name = "Base::ZProviderSrv" 
data_services[name] = CONSTRAINED_BASED_NETWORKS::DataService.new(name)
#
#name = "AuvHelper::DepthAndOrientationFusion" 
#data_services[name] = CONSTRAINED_BASED_NETWORKS::DataService.new(name)
#
#name = "AuvHelper::DepthAndOrientationFusion" 
#tasks[name] = CONSTRAINED_BASED_NETWORKS::Task.new(name).add_fullfillment(name)
#
name = "xsens" 
tasks[name] = CONSTRAINED_BASED_NETWORKS::Task.new(name).add_fullfillment(name).add_fullfillment("Base::OrientationSrv")

name = "depth_sensor" 
tasks[name] = CONSTRAINED_BASED_NETWORKS::Task.new(name).add_fullfillment(name).add_fullfillment("Base::ZProviderSrv")
#
#name = "AuvControl::DepthFusionCmp" 
#compositions[name] = CONSTRAINED_BASED_NETWORKS::Composition.new(name).add_child(tasks["AuvHelper::DepthAndOrientationFusion"],"fusion_child")##.add_child(data_services['Base::ZProviderSrv'],'zprovider_child').add_child(data_services['Base::OrientationSrv'],'ori_child')
#
#name = "AuvControl::DepthFusionCmp2" 
#compositions[name] = CONSTRAINED_BASED_NETWORKS::Composition.new(name).add_child(tasks["AuvHelper::DepthAndOrientationFusion"],"fusion_child")
#
#
#compositions['AuvControl::DepthFusionCmp'].active(true)
#
#s = CONSTRAINED_BASED_NETWORKS::Solution.gist_search
#s = CONSTRAINED_BASED_NETWORKS::Solution.bab_search
#s.print
#
p = CONSTRAINED_BASED_NETWORKS::Pool.instance
p.save("test.dump")


