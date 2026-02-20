pipeline {
  agent none
  options { skipDefaultCheckout(true); ansiColor('xterm') }

  stages {
    stage('Pre-clean workspace') {
      agent { label 'your-jenkins-node-label' }
      steps {
        sh '''
          set -ex
          # remove problematic files even if root-owned (requires sudo rights)
          sudo chown -R jenkins:jenkins "$WORKSPACE" || true
          sudo chmod -R u+rwX,go+rX,go-w "$WORKSPACE" || true
          rm -rf "$WORKSPACE"/*
        '''
      }
    }

    stage('Checkout') {
      agent { label 'your-jenkins-node-label' }
      steps { checkout scm }
    }

    stage('Build in Docker') {
      agent {
        docker {
          image 'slsecrets357/ros-noetic-base:v1'
          args "--entrypoint='' -u root --gpus all --privileged"
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
                which catkin_make; \
                catkin_make -DCMAKE_BUILD_TYPE=Release"
            '''
          }
        }
      }
    }
  }
}