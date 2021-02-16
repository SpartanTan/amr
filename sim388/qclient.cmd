laser connect=localhost:24919
mrc1 fwd 1
mrc1 turn 70
disp scale=3.0 pos=0.25
poollist
poolget img=97 savepng=foo.png
disp help
disp scale=3 pos=0.25
poollist
poolget img=97 savepng=foo.png
laser scanset help
laser scanset logopen
laser scanset log=1
laser odopose log=true
laser scanset logclose
laser odopose log=false
laser scanset logopen
laser scanset log=1
laser odopose log=false
mrc1 fwd 1.5
mrc1 turn 70
laser scanset logclose
laser odopose log=false
laser odopose lof=true
laser odopose log=true
mrc1 fwd 1
laser odopose lof=false
laser odopose log=false
exit
laser scanset logopen
laser scanset log=1
laser odopose log=true
mrc1 fwd 1.5
laser scanset logclose
laser odopose log=false
laser scanset logopen
laser scanset log=1
laser odopose log=true
mrc1 fwd 1.5
turn -70
mrc1 turn -70
scanset logclose
laser scanset logclose
lase odopose log=false
laser odopose log=false
laser push t=0.5 cmd="scanset step=10"
quit
laser push t=0.5 cmd="scanset step=50"
cd ~/mobotware/simulator/trunk/simserver1/
