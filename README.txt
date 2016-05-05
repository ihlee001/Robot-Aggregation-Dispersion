How the program is ran!
Creating the positions for the robots for the .world file
1) radius.cc is the program used to generate the position for each robot
2) run "g++ -o radius radius.cc"
3) then run the executable with the radius given "./radius 140" 
4) It will output a list of positions which is then pasted into the .world file

Running the project3.cc program
1) Run the necessary script to get the executable.  I placed a shortcut for that by just running 
the executable "./compile"
2) It will then output a project3 executable
3) To run aggregation just input "./project3 a 140 50"
4) To run dispersion just input "./project3 d 140 120"
5) input numbers can vary of course

Note:
Make sure to start player first with "player project3.cfg" before running project3.