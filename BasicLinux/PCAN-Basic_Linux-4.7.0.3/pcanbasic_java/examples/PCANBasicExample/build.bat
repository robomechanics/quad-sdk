
set JAVAC_PATH=
set JAVAC_EXE=javac.exe
::
:: Check JAVA in PATH
%JAVAC_EXE% -version
@if not %ERRORLEVEL% == 0 (
    goto main_run
)
::
:: Check JAVA_HOME
if not "%JAVA_HOME%" == "" (
	goto main_skip_java_home
)
set JAVA_HOME=c:\Program Files\Java\jdk-14.0.2
main_skip_java_home:
set JAVAC_PATH=%JAVA_HOME%\bin\

::
:: Main
main_run:
set JAVAC=%JAVAC_PATH%%JAVAC_EXE%
set CLASS_DIR="classes"
set SRC_DIR="src"

dir /s /B %SRC_DIR%\*.java > build_sources.txt
"%JAVAC%" -d %CLASS_DIR%  -Xlint:unchecked @build_sources.txt
pause