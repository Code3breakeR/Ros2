pipeline {
  agent {
    docker {
      image 'osrf/ros:humble-desktop'
      args '-u root:root'
    }
  }

  environment {
    ROS_DISTRO = 'humble'
    ROS_WS = "${env.WORKSPACE}/ros2_ws"
  }

  stages {
    stage('Prepare Workspace') {
      steps {
        sh '''
          echo "Cleaning workspace..."
          rm -rf ${ROS_WS}
          mkdir -p ${ROS_WS}/src
          cd ${ROS_WS}/src
          git clone https://github.com/Code3breakeR/Ros2.git .
        '''
      }
    }

    stage('Install Dependencies') {
      steps {
        sh '''
          apt-get update &&
          apt-get install -y curl gnupg2 lsb-release python3-pip &&
          pip3 install -U colcon-common-extensions &&
          source /opt/ros/${ROS_DISTRO}/setup.sh &&
          rosdep init || true &&
          rosdep update &&
          cd ${ROS_WS} &&
          rosdep install --from-paths src --ignore-src -r -y
        '''
      }
    }

    stage('Build') {
      steps {
        sh '''
          bash -c "
            source /opt/ros/${ROS_DISTRO}/setup.sh &&
            cd ${ROS_WS} &&
            colcon build --symlink-install
          "
        '''
      }
    }

    stage('Deploy to EC2') {
      steps {
        withCredentials([sshUserPrivateKey(credentialsId: 'ec2-key', keyFileVariable: 'KEY')]) {
          sh '''
            echo "Creating ~/.ssh if missing..."
            mkdir -p ~/.ssh
            chmod 700 ~/.ssh
    
            echo "Adding EC2 to known_hosts..."
            ssh-keyscan 43.204.220.247 >> ~/.ssh/known_hosts
    
            echo "Deploying ROS2 install/ folder to EC2..."
            scp -i $KEY -r ros2_ws/install ubuntu@43.204.220.247:/home/ubuntu/ros2_deploy/
          '''
        }
      }
    }


    // Optional testing stages
    // stage('Test') {
    //   steps {
    //     sh '''
    //       bash -c "
    //         source /opt/ros/${ROS_DISTRO}/setup.sh &&
    //         source ${ROS_WS}/install/setup.sh &&
    //         cd ${ROS_WS} &&
    //         colcon test --event-handlers console_direct+
    //       "
    //     '''
    //   }
    // }

    // stage('Test Results') {
    //   steps {
    //     junit allowEmptyResults: true, testResults: '**/Testing/**/*.xml'
    //   }
    // }
  }

  post {
    always {
      sh 'find . > all_files.txt'
      archiveArtifacts artifacts: 'all_files.txt', allowEmptyArchive: true
      archiveArtifacts artifacts: 'ros2_ws/log/**', allowEmptyArchive: true
      archiveArtifacts artifacts: 'ros2_ws/install/**', allowEmptyArchive: true
    }
    success {
      echo '✅ Build and deployment completed successfully.'
    }
    failure {
      echo '❌ Pipeline failed. Check the logs above.'
    }
  }
}

