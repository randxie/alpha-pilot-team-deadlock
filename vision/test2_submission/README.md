## Test 2 Submission
Follow alpha pilot guideline to generate final submisson files

Algorithm
- Test 2 Algorithms need to detect a single polygon per gate in the image. While there will not be multiple gates in any test images in the Eye Exam, there may be some test images with no gates.
- List dependencies in a requirements.txt file - will be installed using pip install -r requirements.txt. (only list libraries needed to be installed for code to run)
    - - Deep Learning AMI instance is used for compilation meaning GPU and CUDA drivers will already be installed 
- Submission code files - 
    - - generate_results.py – Sample submission with example class by which to define a solution. Also reads and plots images and labels.
    - - generate_submission.py – Test script that reads all test images, imports and calls a team’s algorithm, and outputs labels for all images in a JSON file
    - - The testing script, generate_submission.py, will run through all available image data, implement a function call to get the labels and execution time of team algorithms for a given image, and store the results for each image in a JSON file.

Report 
 - Rubric for testing is - technical merit, applicability to AlphaPilot, and presentation and clarity of the approach
 - Proper explanation of why maskrcnn and tradeoff between improved flyable region detection for reduced 
 - Address in technical report how we will extend our algorithm to take into account the possibility that there will be more than 1 gate in view in the later stages
 - Any optimization that we achieved in our algorithm to make it more real-time
