SYSTEM_PROMPT = "You are an excellent interpreter of human instructions for household tasks. Given an instruction and information about the working environment, you break it down into a sequence of robotic primitives."

PRIMITIVES = """DETECT(OBJECT), REACH(OBJECT), CLOSE_GRIPPER(), OPEN_GRIPPER(), TASK_COMPLETED()"""

PRIMITIVES = """1. DETECT(OBJECT) -> OBJECT_POSITION, OBJECT_ORIENTATION : Detects the 3D position and orientation of the object.
2. REACH

"""
PRIMITIVES = """You can complete the task using the following primitives:
1. DETECT(OBJECT or OBJECT_PART) : This function will not return anything, but only print the position, orientation, and dimensions of any object or object part in the environment. This information will be printed for as many instances of the queried object or object part in the environment. If there are multiple objects or object parts to detect, call one function for each object or object part, all before executing any trajectories. The unit is in metres.
2. REACH(OBJECT) : This function will execute the list of trajectory points on the robot arm end-effector, and will also not return anything.
3. CLOSE_GRIPPER() : This function will close the gripper on the robot arm, and will also not return anything.
4. OPEN_GRIPPER() : This function will open the gripper on the robot arm, and will also not return anything.
5. TASK_COMPLETED() : Call this function only when the task has been completed. This function will also not return anything.
6. ASK_HUMAN_FOR_HELP : If you don't know what function to call, you can insert "ASK_HUMAN_FOR_HELP" in the list, and generate the code again.

INITIAL PLANNING 1:
If the task requires interaction with an object part (as opposed to the object as a whole), describe which part of the object would be most suitable for the gripper to interact with.
Then, detect the necessary objects in the environment. Stop generation after this step to wait until you obtain the printed outputs from the detect_object function calls.

INITIAL PLANNING 2:
Then, output Python code to decide which object to interact with, if there are multiple instances of the same object.
Then, describe how best to approach the object (for example, approaching the midpoint of the object, or one of its edges, etc.), depending on the nature of the task, or the object dimensions, etc.
Then, output a detailed step-by-step plan for the trajectory, including when to lower the gripper to make contact with the object, if necessary.
Finally, perform each of these steps one by one. Name each trajectory variable with the trajectory number.
Stop generation after each code block to wait for it to finish executing before continuing with your plan.

You should output a list of the primitives, in the order they should be executed, to complete the task.
But you can also ask for help if you don't know what to do or cannot decompose the task into these primitives.
In that case, you can insert "ASK_HUMAN_FOR_HELP" in the list, and generate the code again.

"""


DESCRIBE_OBJECT = "What does this image describe?"
DESCRIBE_SCENE = "What is happening in this image?"
INSPECT_OBJECT = "What is in this image?"
INSPECT_SCENE = "What is happening in this image?"
INSPECT_OBJECT_INTERACTION = "which objects are interacting with the man?"

INSPECT_FEASIBILTY = """You retrieved this article: {article}. The question is: {question}.
Before even answering the question, consider whether you have sufficient information in the article to answer the question fully.
Your output should JUST be the boolean true or false, of if you have sufficient information in the article to answer the question.
Respond with just one word, the boolean true or false. You must output the word 'True', or the word 'False', nothing else.
"""

PLANNING_HEADER = """Your task is to {task}. You can use the following functions to complete the task: {primitives}.
Your output should be a list of the primitives, in the order they should be executed, to complete the task.
Here is an example of a valid output: ["DETECT(OBJECT)", "REACH(OBJECT)", "CLOSE_GRIPPER()", "TASK_COMPLETED()"]
However, if you don't know what function to call, you can insert "ASK_HUMAN_FOR_HELP" in the list, and generate the code again.
"""

INSPECT_FEASIBILTY = """Your task is to {task}. You can use the following functions to complete the task:
you have to output the final code yourself by making use of the available information, common sense, and general knowledge.
You are able to call any of the following Python functions listed in AVAILABLE FUNCTIONS, as often as you want.

AVAILABLE FUNCTIONS:
1. detect_object(object_or_object_part: str) -> None: This function will not return anything, but only print the position, orientation, and dimensions of any object or object part in the environment. This information will be printed for as many instances of the queried object or object part in the environment. If there are multiple objects or object parts to detect, call one function for each object or object part, all before executing any trajectories. The unit is in metres.
2. execute_trajectory(trajectory: list) -> None: This function will execute the list of trajectory points on the robot arm end-effector, and will also not return anything.
3. open_gripper() -> None: This function will open the gripper on the robot arm, and will also not return anything.
4. close_gripper() -> None: This function will close the gripper on the robot arm, and will also not return anything.
5. task_completed() -> None: Call this function only when the task has been completed. This function will also not return anything.
When calling any of the functions, make sure to stop generation after each function call and wait for it to be executed, before calling another function and continuing with your plan.
However, if you don't know what function to call, you can ask the robot to perform a task, and the robot will generate the


CODE GENERATION:
When generating the code for the trajectory, do the following:
1. Describe briefly the shape of the motion trajectory required to complete the task.
2. The trajectory could be broken down into multiple steps. In that case, each trajectory step (at default speed) should contain at least 100 points. Define general functions which can be reused for the different trajectory steps whenever possible, but make sure to define new functions whenever a new motion is required. Output a step-by-step reasoning before generating the code.
3. If the trajectory is broken down into multiple steps, make sure to chain them such that the start point of trajectory_2 is the same as the end point of trajectory_1 and so on, to ensure a smooth overall trajectory. Call the execute_trajectory function after each trajectory step.
4. When defining the functions, specify the required parameters, and document them clearly in the code. Make sure to include the orientation parameter.
5. If you want to print the calculated value of a variable to use later, make sure to use the print function to three decimal places, instead of simply writing the variable name. Do not print any of the trajectory variables, since the output will be too long.
6. Mark any code clearly with the ```python and ``` tags.


When calling any of the functions, make sure to stop generation after each function call and wait for it to be executed, before calling another function and continuing with your plan.
Before even answering the question, consider whether you have sufficient information in the article to answer the question fully.
Your output should JUST be the boolean true or false, of if you have sufficient information in the article to answer the question.
Respond with just one word, the boolean true or false. You must output the word 'True', or the word 'False', nothing else.
"""


PLANNING_HEADER = """Possible primitive tasks are ["move_to", "follow", "grasp", "pass_to", "speak", "answer"]

Q. Follow Robin from the couch to the corridor
A. [["move_to", "the couch"], ["follow", "Robin"], ["move_to", "the corridor"]]

Q. Follow Charlie
A. [["follow", "Charlie"]]

Q. Follow Robin to the bedroom
A. [["follow", "Robin"], ["move_to", "the bedroom"]

Q. Robot please meet Francis at the bed, follow her, and escort her back
A. [["move_to", "the bed"], ["follow", "Francis"]]

Q. Go to the dining room, find John at the table and answer his question.
A. [["move_to", "the diningroom"], ["move_to", "the table"], ["move_to", "John"], ["answer", "question"]]

Q. Robot please take the orange from the storage table and bring it to Alex at the couch
A. [["move_to", "the storage table"], ["grasp", "the orange"], ["move_to", "the couch"], ["move_to", "Alex"], ["pass_to", "Alex"]]

Q. Bring me the object above the sponge from the desk
A. [["move_to", "the desk"], ["move_to", "the object above the sponge"], ["grasp", "the object above the sponge"], ["move_to", "me"], ["pass_to", "me"]]

Q. Could you please meet Michael at the dining table, follow him, and escort him back
A. [["move_to", "the dining table"], ["move_to", "Michael"], ["follow", "Michael"]]

Q. Navigate to the dishwasher, meet Robert, and lead him to the sink
A. [["move_to", "the dishwasher"], ["move_to", "Robert"], ["speak", "Please follow me."], ["move_to", "the sink"]]

Q. Could you give drinks to all the elders in the living room
A. [["move_to", "the living room"], ["move_to", "drinks"], ["grasp", "drinks"], ["move_to", "all the elders"], ["pass_to", "all the elders"]]

Q. Look for the fruits in the corridor
A. [["move_to", "the corridor"], ["move_to", "the fruits"], ["speak", "I found the fruits."]]

Q. Dump the garbage
A. [["move_to", "the garbage"], ["grasp", "the garbage"], ["move_to", "the trash"], ["pass_to", "the trash"]]

Q. Give me the left most object from the dining table
A. [["move_to", "the dining table"], ["move_to", "the left most object from the dining table"]]

Q. Meet James at the bed, follow him, and go to the living room
A. [["move_to", "the bed"], ["move_to", "James"], ["follow", "James"], ["move_to", "the living room"]]

Q. Serve snacks to everyone in the bedroom
A. [["move_to", "the bedroom"], ["move_to", "snacks"], ["grasp", "snacks"], ["move_to", "everyone"], ["pass_to", "everyone"]]

Q. Guide Alex from the entrance to the sink
A. [["move_to", "the entrance"], ["move_to", "Alex"], ["speak", "Please follow me."], ["move_to", "the sink"]]

Q. Give me the biggest object from the cupboard
A. [["move_to", "the cupboard"], ["move_to", "the biggest object from the cupboard"], ["grasp", "the biggest object from the cupboard"], ["move_to", "me"], ["pass_to", "me"]]

Q. Greet Charlie at the dishwasher and introduce it to Robert at the desk
A. [["move_to", "the dishwasher"], ["move_to", "Charlie"], ["speak", "Hello."], ["speak", "Please follow me."], ["move_to", "the desk"], ["move_to", "Robert"], ["speak", "This is Charlie"]]

Q. Meet Robin at the bed, follow him, and navigate to the bedroom
A. [["move_to", "the bed"], ["move_to", "Robin"], ["follow", "Robin"], ["move_to", "the bedroom"]]

Q. Could you please escort Charlie to the sink
A. [["move_to", "Charlie"], ["speak", "Please follow me."], ["move_to", "the sink"]]

Q. Please tell me which are the three thinnest cutlery on the side table
A. [["move_to", "Charlie"], ["speak", "Please follow me."], ["move_to", "the sink"]]

Q. Go to the kitchen, find James at the sink and answer his question.
A. [["move_to", "the kitchen"], ["move_to", "the sink"], ["move_to", "James"], ["answer", "question"]]

Q. Could you face Francis at the exit and introduce it to Robin at the sink
A. [["move_to", "the exit"], ["move_to", "Francis"], ["speak", "Please follow me."], ["move_to", "the sink"], ["move_to", "Robin"], ["speak", "This is Francis"]]

Q. Look for the orange in the dining room
A. [["move_to", "the dining room"], ["move_to", "the orange"], ["speak", "I found the orange."]]

Q. Meet Francis and ask her what's your name
A. [["move_to", "Francis"], ["speak", "what's your name"]]

Please answer the below question as a nested python list without any decoration such as "Answer: " and "A".
"""


PLANNING_HEADER = """Your task is to {task}. You can use the following functions to complete the task: {primitives}.
Your output should be a list of the primitives, in the order they should be executed, to complete the task.
Here is an example of a valid output: ["DETECT(OBJECT)", "REACH(OBJECT)", "CLOSE_GRIPPER()", "TASK_COMPLETED()"]
However, if you don't know what function to call, you can insert "ASK_HUMAN_FOR_HELP" in the list, and generate the code again.
"""
