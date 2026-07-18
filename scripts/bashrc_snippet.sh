# Add this snippet to ~/.bashrc on the Ubuntu 20.04 competition PC.

source /opt/ros/noetic/setup.bash

if [ -f "$HOME/morai_competition_ws/devel/setup.bash" ]; then
  source "$HOME/morai_competition_ws/devel/setup.bash"
fi

export MORAI_COMPETITION_WS="$HOME/morai_competition_ws"
export MORAI_COMPETITION_LOG_DIR="$HOME/morai_competition_logs"

alias cw='cd "$MORAI_COMPETITION_WS"'
alias cs='cd "$MORAI_COMPETITION_WS/src"'
alias cm='cd "$MORAI_COMPETITION_WS" && catkin_make'
alias rl='rostopic list'
alias rgp='rqt_graph'
alias morai_run='roslaunch morai_competition_bringup competition.launch'
alias morai_run_log='roslaunch morai_competition_bringup competition.launch record_bag:=true'

