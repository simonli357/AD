pipeline {
    agent {
        docker {
            image 'slsecrets357/ros-noetic-5090-base:v1'
            args '-u root --entrypoint="" --gpus all --privileged'
        }
    }

    options {
        ansiColor('xterm')
    }

    stages {
        stage('Clean') {
            steps {
                sh '''
                    rm -rf build devel install
                '''
            }
        }

        stage('Dependencies') {
            steps {
                sh '''
                    if [ -f requirements.txt ]; then
                        pip3 install -r requirements.txt
                    fi
                '''
            }
        }

        stage('Build') {
            steps {
                sh '''
                bash -lc 'source /opt/ros/noetic/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release'
                '''
            }
        }
    }
}