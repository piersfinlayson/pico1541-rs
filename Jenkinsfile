pipeline {
    agent {
        dockerfile {
            filename 'Dockerfile.jenkins'
        }
    }
    
    stages {
        stage('Build') {
            steps {
                sh 'bash ./build.sh all all'
                sh 'bash ./build.sh all all release'
            }
        }
    }
}