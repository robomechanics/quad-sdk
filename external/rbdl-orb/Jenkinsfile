pipeline {
  agent any

  stages {
    stage('Build RBDL') {
      steps {
        sh 'git submodule update --init --recursive'
        cmakeBuild(installation: 'InSearchPath', buildDir: 'build', buildType: 'Release', cleanBuild: true, cmakeArgs: '-DRBDL_BUILD_ADDON_BENCHMARK=ON -DRBDL_BUILD_ADDON_GEOMETRY=ON -DRBDL_BUILD_ADDON_LUAMODEL=ON -DRBDL_BUILD_ADDON_MUSCLE=ON -DRBDL_BUILD_ADDON_MUSCLE_FITTING=ON -DRBDL_BUILD_ADDON_URDFREADER=ON -DRBDL_BUILD_PYTHON_WRAPPER=ON -DRBDL_BUILD_TESTS=ON')
        dir(path: 'build') {
          sh 'make -j 8'
        }
      }
    }

    stage('Test RBDL') {
      steps {
        dir(path: 'build') {
          sh './tests/rbdl_tests -r junit > results_rbdl.xml'
          sh './addons/geometry/tests/geometry_tests -r junit > results_geometry.xml'
          sh './addons/muscle/tests/muscle_tests -r junit -o ./results_muscle.xml'
          sh './addons/luamodel/tests/luamodel_tests -r junit > results_luamodel.xml'
        }
        dir(path: 'build/python') {
          sh './test_rbdlmuscle.py -v'
          sh './test_wrapper.py -v'
        }
      }
    }
  }

  post {
    always {
      junit 'build/results_rbdl.xml'
      junit 'build/results_geometry.xml'
      junit 'build/results_muscle.xml'
      junit 'build/results_luamodel.xml'
    }
  }
}
