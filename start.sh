#!/bin/bash
# Demo repo and demo directory path on the sbc here


REPO="https://github.com/FARI-brussels/demo-fari-brickiebot"

DIR="/home/fari/Documents/demo-fari-brickiebot"


SCRIPT_DIR="/home/fari/Documents/TE-Scripts"
# Use git_sync.sh to sync both repositories
"$SCRIPT_DIR/clone_or_pull_repo.sh" "$DIR" "$REPO"



# Set the correct Node.js version using nvm
export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"  # This loads nvm
[ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"  # This loads nvm bash_completion

# Use a specific Node.js version
nvm use node


#xinput set-prop 11 'Coordinate Transformation Matrix' 0 -0.5 1 1 0 0 0 0 1
#xinput set-prop 6 'Coordinate Transformation Matrix' 0 -0.5 0.5 1 0 0 0 0 1


# Remove chromium cache
rm -rf ~/.cache/chromium

# Kill any process using port 5173 (if running)
kill -9 $(lsof -t -i:5173)

# Navigate to the demo directory and run npm install
cd "$DIR"
npm install

#run backend
#gnome-terminal --working-directory=$BACKEND_DIR -- bash -c "source /home/fari/miniconda3/etc/profile.d/conda.sh && conda activate tictactoe && python main.py --modes REAL;"


#run frontend
gnome-terminal --working-directory="$DIR"/frontend -- bash -c "npm run dev; read -p 'Press enter to continue...'"
gnome-terminal --working-directory="$DIR"/controller -- bash -c "source /home/fari/miniconda3/etc/profile.d/conda.sh && conda activate base && python xbox_controller.py; read -p 'Press enter to continue...'"

gnome-terminal --working-directory="$DIR"/simulation -- bash -c "source /home/fari/miniconda3/etc/profile.d/conda.sh && conda activate base && python simulation.py; read -p 'Press enter to continue...'"

gnome-terminal -- bash -c "chromium 'http://localhost:5173/'; read -p 'Press enter to continue...'"
sleep 3

xdotool search --onlyvisible --class chromium windowmove 3000 0

sleep 1
xdotool search --onlyvisible --class mujoco 

xdotool search --onlyvisible --class mujoco windowmove 0 0
sleep 2
xdotool search --onlyvisible --class mujoco windowsize 100% 100%

