# CMake generated Testfile for 
# Source directory: /home/osher/Dropbox/Work/CGP/Assignments/CGP1/cgp1-prep/test
# Build directory: /home/osher/Dropbox/Work/CGP/Assignments/CGP1/cgp1-prep/test
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
ADD_TEST(smoketest "/home/osher/Dropbox/Work/CGP/Assignments/CGP1/cgp1-prep/test/tilertest" "-v" "--test=commit")
SET_TESTS_PROPERTIES(smoketest PROPERTIES  WORKING_DIRECTORY "/home/osher/Dropbox/Work/CGP/Assignments/CGP1/cgp1-prep")
