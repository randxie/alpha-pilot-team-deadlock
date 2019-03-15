## Test 3 Submission files
## Official files do not work. Follow the steps below using modified files
* Copy scorer.launch into ../flightlaunch directory.
* Copy scorer.sh into ../flightgoggles/flightgoggles directory, and do 'chmod +x scorer.sh'.
* Copy both the /challenges and /perturbations folder into your ../config directory.
* rosrun flightgoggles scorer.sh (**make sure scorer.py is in the same directory when you rosrun**)
* Delete the /results folder generated scorer.sh, because it will try to mkdir that folder again and will cause an error.


See following for reference on the issues:
1. https://github.com/mit-fast/FlightGoggles/issues/80
2. See Adriano Rezende's post https://www.herox.com/alphapilot/forum/thread/3755?page=11

