pipeline {
  agent {
    docker {
        image 'slsecrets357/ros-noetic-base:v1'
        args "--entrypoint='' -u root --gpus all --privileged"
        alwaysPull true
    }
  }

  options { ansiColor('xterm') }

  stages {
    stage('Clean') {
      steps { sh 'rm -rf build devel install' }
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
            bash -lc "set -ex; \
                source /opt/ros/noetic/setup.bash; \
                which catkin_make; \
                catkin_make -DCMAKE_BUILD_TYPE=Release"
            '''
        }
    }
  }
}
