pipeline {
    agent {
        docker { 
            image 'libmcsim-build-env:1' 
            args '-v /var/www/html/jenkins/:/var/www/html/jenkins/'
        }
    }

    environment {
        RECIPIENT_LIST = 'dev@marekcel.pl'
    }

    triggers {
        pollSCM('0 3 * * 1-5')
    }

    options {
        skipStagesAfterUnstable()
    }

    stages {
        stage('Build') {
            steps {
                sh 'cd misc; python3 ./build.py --with-tests'
            }
        }
        stage('Test') {
            steps {
                sh 'cd misc; python3 ./run_tests.py'
            }
        }
        stage('Generate coverage report') {
            steps {
                sh 'cd misc; python3 ./generate_coverage-report.py'
                sh "mkdir -p /var/www/html/jenkins/coverage-reports/${env.JOB_NAME}"
                sh "cp -r coverage-report /var/www/html/jenkins/coverage-reports/${env.JOB_NAME}/build-${env.BUILD_NUMBER}"
            }
        }
    }

    post {
        success {
            script {
                def buildDate = new Date(currentBuild.startTimeInMillis).format("yyyy-MM-dd")
                env.BUILD_DATE = buildDate
            }
            emailext (
                to: "${env.RECIPIENT_LIST}",
                subject: "SUCCESS: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]'",
                body: """<p>SUCCESS: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]':</p>
                <p>Check console output at <a href='${env.BUILD_URL}'>${env.BUILD_URL}</a></p>
                <p>Check coverage report at <a href='http://192.168.100.102/jenkins/coverage-reports/${env.JOB_NAME}/build-${env.BUILD_NUMBER}'>http://192.168.100.102/jenkins/coverage-reports/${env.JOB_NAME}/build-${env.BUILD_NUMBER}</a></p>""",
                mimeType: 'text/html'
            )
        }

        failure {
            emailext (
                to: "${env.RECIPIENT_LIST}",
                subject: "FAILURE: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]'",
                body: """<p>FAILURE: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]':</p>
                <p>Check console output at <a href='${env.BUILD_URL}'>${env.BUILD_URL}</a></p>
                <p>Check coverage report at <a href='http://192.168.100.102/jenkins/coverage-reports/${env.JOB_NAME}/build-${env.BUILD_NUMBER}'>http://192.168.100.102/jenkins/coverage-reports/${env.JOB_NAME}/build-${env.BUILD_NUMBER}</a></p>""",
                mimeType: 'text/html'
            )
        }
    }
}
