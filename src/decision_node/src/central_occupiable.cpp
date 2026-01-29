#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include "decision_node/central_occupiable.hpp"

#include <ros/ros.h>  

class AccumulateCentralOccupiable : public BT::SyncActionNode
{
public:
    AccumulateCentralOccupiable(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        ros::NodeHandle nh("~");  
        
        //set default threshold in case the server parameter has sth. wrong
        int default_threshold = 20;
        nh.param("central_threshold", default_threshold, default_threshold);
        
        threshold_ = default_threshold;
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("occupy_status"),
            BT::InputPort<int>("threshold"),
            BT::OutputPort<int>("accumulated_count"),
            BT::OutputPort<bool>("reached_threshold")
        };
    }

    BT::NodeStatus tick() override
    {
        auto bb = config().blackboard;
        
        int occupy_status;
        
        if (!getInput("occupy_status", occupy_status))
        {
            return BT::NodeStatus::FAILURE;
        }
        
        int current_count = 0;
        try {
            current_count = bb->get<int>("central_accumulate_count");
        } catch (...) {
            current_count = 0;
        }
        
        bool reached = false;
        bool is_enemy_occupied = (occupy_status == 2); // 假设2是敌方占领
        
        if (!is_enemy_occupied)
        {
            current_count++;
            if (current_count >= threshold_)
            {
                reached = true;
                current_count = 0;
            }
        }
        else
        {
            current_count = 0;
        }
        
        
        bb->set("central_accumulate_count", current_count);
        setOutput("accumulated_count", current_count);
        setOutput("reached_threshold", reached);
        
        return BT::NodeStatus::SUCCESS;
    }

private:
    int threshold_;  
};

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

class CheckEnemyOccupied : public BT::ConditionNode
{
public:
    CheckEnemyOccupied(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ConditionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("occupy_status")
        };
    }

    BT::NodeStatus tick() override
    {
        int occupy_status;
        if (!getInput("occupy_status", occupy_status))
        {
            return BT::NodeStatus::FAILURE;
        }
        
        // 假设2是敌方占领
        bool is_enemy_occupied = (occupy_status == 2);
        return is_enemy_occupied ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
};

void RegisterOccupationNodes(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<TriggerOnThreshold>("TriggerOnThreshold");
    factory.registerNodeType<ResetAccumulator>("ResetAccumulator");
    factory.registerNodeType<CheckEnemyOccupied>("CheckEnemyOccupied");
}

void RegisterAccumulateCentralOccupiable(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<AccumulateCentralOccupiable>("AccumulateCentralOccupiable");
}

void RegisterTriggerOnThreshold(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<TriggerOnThreshold>("TriggerOnThreshold");
}

void RegisterResetAccumulator(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<ResetAccumulator>("ResetAccumulator");
}

void RegisterCheckEnemyOccupied(BT::BehaviorTreeFactory& factory)
{
    factory.registerNodeType<CheckEnemyOccupied>("CheckEnemyOccupied");
}