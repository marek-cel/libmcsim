pipeline {
    agent any

    environment {
        RECIPIENT_LIST = 'dev@marekcel.pl'
    }

    triggers {
        pollSCM('0 3 * * 1-5')
    }

    stages {
        stage('Build') {
            steps {
                sh 'cmake -B ./build -DTESTING=On'
                sh 'cd build; make -j4'
            }
        }
        stage('Test'){
            steps {
                sh './run_tests.sh'
            }
        }
        stage('Generate coverage report'){
            steps {
                sh './generate_coverage-report.sh'
                sh "mkdir -p $JENKINS_HOME/userContent/libmcsim/coverage-reports"
                sh "cp -r coverage-report $JENKINS_HOME/userContent/libmcsim/coverage-reports/\$(date +%Y-%m-%d)_build-${env.BUILD_NUMBER}"
            }
        }
    }


    post {
        success {
            emailext (
                to: "${env.RECIPIENT_LIST}",
                subject: "SUCCESS: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]'",
                body: """<p>SUCCESS: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]':</p>
                <p>Check console output at <a href='${env.BUILD_URL}'>${env.JOB_NAME} [${env.BUILD_NUMBER}]</a></p>""",
                mimeType: 'text/html'
            )
        }

        failure {
            emailext (
                to: "${env.RECIPIENT_LIST}",
                subject: "FAILURE: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]'",
                body: """<p>FAILURE: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]':</p>
                <p>Check console output at <a href='${env.BUILD_URL}'>${env.JOB_NAME} [${env.BUILD_NUMBER}]</a></p>""",
                mimeType: 'text/html'
            )
        }
    }
}
