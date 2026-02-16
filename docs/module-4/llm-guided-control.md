---
sidebar_position: 3
title: "LLM-Guided Robot Control"
description: "Using large language models to plan and control robot behavior through natural language."
---

# LLM-Guided Robot Control

> *"Tell the robot what to do in plain English — the LLM figures out the how."*

## The LLM-Robot Pipeline

```
Human: "Can you tidy up the living room?"
                │
                ▼
┌──────────────────────────────┐
│  LLM Task Planner (GPT-4o)   │
│                               │
│  1. Scan room for objects     │
│  2. Identify misplaced items  │
│  3. Pick up each item         │
│  4. Place in correct location │
│  5. Confirm completion        │
└──────────────┬───────────────┘
               │
               ▼
┌──────────────────────────────┐
│  Action Executor (ROS 2)      │
│                               │
│  navigate_to(living_room)     │
│  scan_environment()           │
│  pick_up("book")             │
│  place_on("bookshelf")       │
│  pick_up("cup")              │
│  place_on("kitchen_counter")  │
│  say("Done tidying up!")     │
└──────────────────────────────┘
```

## Code-as-Actions (SayCan Approach)

```python
class SayCanPlanner:
    """
    SayCan: 'I can do X, should I?'
    Combines LLM knowledge (should I?) with robot capability (can I?).
    """
    
    def __init__(self, llm_client, robot):
        self.client = llm_client
        self.robot = robot
        
        # Available skills
        self.skills = {
            "pick_up": self.robot.pick_up,
            "place_on": self.robot.place_on,
            "navigate_to": self.robot.navigate_to,
            "open": self.robot.open_container,
            "pour": self.robot.pour,
        }
    
    def plan_and_execute(self, instruction):
        """Plan and execute a natural language instruction."""
        
        # Get current scene description
        scene = self.robot.describe_scene()
        objects = self.robot.detect_objects()
        
        # Ask LLM for plan
        plan = self.get_plan(instruction, scene, objects)
        
        # Execute each step with safety checks
        for step in plan:
            # Check affordance (can the robot physically do this?)
            if self.robot.can_execute(step):
                success = self.execute_step(step)
                if not success:
                    # Re-plan on failure
                    plan = self.replan(instruction, step, "execution failed")
            else:
                # Skill not possible — ask LLM for alternative
                plan = self.replan(instruction, step, "not feasible")
    
    def get_plan(self, instruction, scene, objects):
        response = self.client.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": f"""You are a robot planner.
                Scene: {scene}
                Detected objects: {objects}
                Available skills: {list(self.skills.keys())}
                
                Output a JSON array of steps."""},
                {"role": "user", "content": instruction}
            ],
            response_format={"type": "json_object"},
        )
        return json.loads(response.choices[0].message.content)["steps"]
```

## Inner Monologue: Self-Correcting Robots

```python
class InnerMonologue:
    """Robot that reasons about its own success/failure."""
    
    def execute_with_reasoning(self, task):
        plan = self.plan(task)
        
        for step in plan:
            # Execute
            self.execute(step)
            
            # Check success (use vision model)
            image = self.camera.capture()
            success_check = self.check_success(image, step)
            
            if not success_check['success']:
                # Inner monologue
                reasoning = self.reason_about_failure(
                    step, success_check['observation']
                )
                
                # Example reasoning:
                # "I tried to pick up the cup, but my gripper closed on
                # empty air. The cup appears to have moved. I should
                # re-detect the cup's position and try again."
                
                # Retry with adjusted approach
                adjusted_step = self.adjust_action(step, reasoning)
                self.execute(adjusted_step)
```

## Summary

LLM-guided robot control enables natural language instruction of robots, with the LLM handling high-level reasoning and planning while traditional robotics handles low-level execution.

---

**Next:** [End-to-End Learning →](/docs/module-4/end-to-end-learning)
