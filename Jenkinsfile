pipeline {
    agent any
    triggers { cron('5 3 * * 1-5') }
    options {
        skipStagesAfterUnstable()
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
}
