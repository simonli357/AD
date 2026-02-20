pipeline {
  agent none
  options { skipDefaultCheckout(true); ansiColor('xterm') }

  stages {
    stage('Pre-clean workspace') {
      agent any
      steps {
        sh '''
          set -ex
          rm -rf "$WORKSPACE"/*
        '''
      }
    }

    stage('Checkout') {
      agent any
      steps { checkout scm }
    }

    stage('Build') {
      agent {
        docker {
          image 'slsecrets357/ros-noetic-base:v1'
          args "--entrypoint='' -u 132:138 --gpus all --privileged"
          alwaysPull true
        }
      }
      stages {
        stage('Clean') { steps { sh 'rm -rf build devel install' } }
        stage('Dependencies') {
          steps { sh 'test -f requirements.txt && pip3 install -r requirements.txt || true' }
        }
        stage('Build') {
          steps {
            sh '''
              bash -lc "set -ex; \
                source /opt/ros/noetic/setup.bash; \
                catkin_make -DCMAKE_BUILD_TYPE=Release"
            '''
          }
        }
      }
    }
  }
}