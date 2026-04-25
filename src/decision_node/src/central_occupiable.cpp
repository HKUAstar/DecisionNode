#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include "decision_node/central_occupiable.hpp"

#include <ros/ros.h>  

class TriggerOnThreshold : public BT::ConditionNode
{
public:
    TriggerOnThreshold(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config), has_triggered_(false)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<bool>("reached_threshold"),
            BT::InputPort<bool>("reset_condition")
        };
    }

    BT::NodeStatus tick() override
    {
        bool reached_threshold = false;
        bool reset_condition = false;
        
        getInput("reached_threshold", reached_threshold);
        getInput("reset_condition", reset_condition);
        
        
        if (reset_condition)
        {
            has_triggered_ = false;
        }
        
        
        if (reached_threshold && !has_triggered_)
        {
            has_triggered_ = true;
            return BT::NodeStatus::SUCCESS;  
        }
        
        return BT::NodeStatus::FAILURE;
    }

private:
    bool has_triggered_;  
};

class ResetAccumulator : public BT::SyncActionNode
{
public:
    ResetAccumulator(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};  
    }

    BT::NodeStatus tick() override
    {
        auto bb = config().blackboard;
        
    
        bb->set("central_accumulate_count", 0);
        
    
        try {
            bb->set("central_occupiable_triggered", false);
        } catch (...) {
            
        }
        
        return BT::NodeStatus::SUCCESS;
    }
};

void RegisterOccupationNodes(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<TriggerOnThreshold>("TriggerOnThreshold");
    factory.registerNodeType<ResetAccumulator>("ResetAccumulator");
}

void RegisterTriggerOnThreshold(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<TriggerOnThreshold>("TriggerOnThreshold");
}

void RegisterResetAccumulator(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<ResetAccumulator>("ResetAccumulator");
}

