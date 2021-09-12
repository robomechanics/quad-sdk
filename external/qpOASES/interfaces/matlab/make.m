function [] = make( varargin )
%MAKE Compiles the Matlab interface of qpOASES.
%
%Type  make            to compile all interfaces that 
%                      have been modified,
%type  make clean      to delete all compiled interfaces,
%type  make clean all  to first delete and then compile 
%                      all interfaces,
%type  make 'name'     to compile only the interface with
%                      the given name (if it has been modified),
%type  make 'opt'      to compile all interfaces using the 
%                      given compiler options.
%
%Copyright (C) 2013-2017 by Hans Joachim Ferreau, Andreas Potschka,
%Christian Kirches et al. All rights reserved.

%%
%%	This file is part of qpOASES.
%%
%%	qpOASES -- An Implementation of the Online Active Set Strategy.
%%	Copyright (C) 2007-2017 by Hans Joachim Ferreau, Andreas Potschka,
%%	Christian Kirches et al. All rights reserved.
%%
%%	qpOASES is free software; you can redistribute it and/or
%%	modify it under the terms of the GNU Lesser General Public
%%	License as published by the Free Software Foundation; either
%%	version 2.1 of the License, or (at your option) any later version.
%%
%%	qpOASES is distributed in the hope that it will be useful,
%%	but WITHOUT ANY WARRANTY; without even the implied warranty of
%%	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
%%	See the GNU Lesser General Public License for more details.
%%
%%	You should have received a copy of the GNU Lesser General Public
%%	License along with qpOASES; if not, write to the Free Software
%%	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
%%

%%
%%	Filename:  interfaces/matlab/make.m
%%	Author:    Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
%%	Version:   3.2
%%	Date:      2007-2017
%%

       
    %% consistency check
    if ( exist( [pwd, '/make.m'],'file' ) == 0 )
        error( ['ERROR (',mfilename '.m): Run this make script directly within the directory ', ...
                '<qpOASES-inst-dir>/interfaces/matlab, please.'] );
    end


    if ( nargin > 2 )
        error( ['ERROR (',mfilename '.m): At most two make arguments supported!'] );
    else
        [ doClean,fcnNames,userFlags ] = analyseMakeArguments( nargin,varargin );
    end

    
    %% define compiler settings
    QPOASESPATH = '../../';
    
    DEBUGFLAGS = ' ';
    %DEBUGFLAGS = ' -v -g CXXDEBUGFLAGS=''$CXXDEBUGFLAGS -Wall -pedantic -Wshadow'' ';

    IFLAGS = [ '-I. -I',QPOASESPATH,'include',' -I',QPOASESPATH,'src',' ' ];
    CPPFLAGS = [ IFLAGS, DEBUGFLAGS, '-largeArrayDims -D__cpluplus -D__MATLAB__ -D__AVOID_LA_NAMING_CONFLICTS__ -D__SINGLE_OBJECT__',' ' ];
    defaultFlags = '-O -D__NO_COPYRIGHT__ '; %% -D__SUPPRESSANYOUTPUT__

    if ( ispc() == 0 )
        CPPFLAGS = [ CPPFLAGS, '-DLINUX -lmwblas',' ' ];
    else
        CPPFLAGS = [ CPPFLAGS, '-DWIN32',' ' ];
    end

    if ( isempty(userFlags) > 0 )
        CPPFLAGS = [ CPPFLAGS, defaultFlags,' ' ];
    else
        CPPFLAGS = [ CPPFLAGS, userFlags,' ' ];
    end

	%% determine if MA57 is available for sparse linear algebra
	isoctave = exist('OCTAVE_VERSION', 'builtin') ~= 0;
	if isoctave
		warning('Sparse linear algebra is currently not available for qpOASES in Octave. Passing sparse matrices works but will likely be slow.')
		SPARSEFLAGS = '';
	elseif verLessThan('matlab', '7.8')
		warning('Sparse linear algebra is currently available for qpOASES only for Matlab versions 7.8 and later. Passing sparse matrices works but will likely be slow.')
		SPARSEFLAGS = '';
	else
		if ( ispc() == 0 )
			SPARSEFLAGS = '-largeArrayDims -D__USE_LONG_INTEGERS__ -D__USE_LONG_FINTS__ -DSOLVER_MA57 -lmwma57 ';
		else
			SPARSEFLAGS = '-largeArrayDims -D__USE_LONG_INTEGERS__ -D__USE_LONG_FINTS__ ';
		end
	end

    mexExt = eval('mexext');
    
    
    %% ensure copyright notice is displayed
    if ~isempty( strfind( CPPFLAGS,'-D__NO_COPYRIGHT__' ) )
        printCopyrightNotice( );
    end
    
    
    %% clean if desired
    if ( doClean > 0 )
        
        eval( 'delete *.o;' );
        eval( ['delete *.',mexExt,'*;'] );
        disp( [ 'INFO (',mfilename '.m): Cleaned all compiled files.'] );
        pause( 0.2 );
        
    end
    
    
    if ( ~isempty(userFlags) )
        disp( [ 'INFO (',mfilename '.m): Compiling all files with user-defined compiler flags (''',userFlags,''')...'] );
    end
    

    %% call mex compiler
    for ii=1:length(fcnNames)
        
        cmd = [ 'mex -output ', fcnNames{ii}, ' ', CPPFLAGS, SPARSEFLAGS, [fcnNames{ii},'.cpp'] ];
        
        if ( exist( [fcnNames{ii},'.',mexExt],'file' ) == 0 )
            
            eval( cmd );
            disp( [ 'INFO (',mfilename '.m): ', fcnNames{ii},'.',mexExt, ' successfully created.'] );
            
        else
            
            % check modification time of source/Make files and compiled mex file
            cppFile = dir( [pwd,'/',fcnNames{ii},'.cpp'] );
            cppFileTimestamp = getTimestamp( cppFile );
            
            utilsFile = dir( [pwd,'/qpOASES_matlab_utils.cpp'] );
            utilsFileTimestamp = getTimestamp( utilsFile );
            
            makeFile = dir( [pwd,'/make.m'] );
            makeFileTimestamp = getTimestamp( makeFile );
            
            mexFile = dir( [pwd,'/',fcnNames{ii},'.',mexExt] );
            if ( isempty(mexFile) == 0 )
                mexFileTimestamp = getTimestamp( mexFile );
            else
                mexFileTimestamp = 0;
            end
            
            if ( ( cppFileTimestamp   >= mexFileTimestamp ) || ...
                 ( utilsFileTimestamp >= mexFileTimestamp ) || ...
                 ( makeFileTimestamp  >= mexFileTimestamp ) )
                eval( cmd );
                disp( [ 'INFO (',mfilename '.m): ', fcnNames{ii},'.',mexExt, ' successfully created.'] );
            else            
                disp( [ 'INFO (',mfilename '.m): ', fcnNames{ii},'.',mexExt, ' already exists.'] );
            end
            
        end
        
    end

    %% add qpOASES directory to path
    path( path,pwd );

end


function [ doClean,fcnNames,userIFlags ] = analyseMakeArguments( nArgs,args )

    doClean = 0;
    fcnNames = [];
    userIFlags = [];

    switch ( nArgs )
        
        case 1
            if ( strcmp( args{1},'all' ) > 0 )
                fcnNames = { 'qpOASES','qpOASES_sequence' };
            elseif ( strcmp( args{1},'qpOASES' ) > 0 )
                fcnNames = { 'qpOASES' };
            elseif ( strcmp( args{1},'qpOASES_sequence' ) > 0 )
                fcnNames = { 'qpOASES_sequence' };
            elseif ( strcmp( args{1},'clean' ) > 0 )
                doClean = 1;
            elseif ( strcmp( args{1}(1),'-' ) > 0 )
                % make clean all with user-specified compiler flags
                userIFlags = args{1};
                doClean = 1;
                fcnNames = { 'qpOASES','qpOASES_sequence' };
            else
                error( ['ERROR (',mfilename '.m): Invalid first argument (''',args{1},''')!'] );
            end

        case 2
            if ( strcmp( args{1},'clean' ) > 0 )
                doClean = 1;
            else
                error( ['ERROR (',mfilename '.m): First argument must be ''clean'' if two arguments are provided!'] );
            end
            
            if ( strcmp( args{2},'all' ) > 0 )
                fcnNames = { 'qpOASES','qpOASES_sequence' };
            elseif ( strcmp( args{2},'qpOASES' ) > 0 )
                fcnNames = { 'qpOASES' };
            elseif ( strcmp( args{2},'qpOASES_sequence' ) > 0 )
                fcnNames = { 'qpOASES_sequence' };
            else
                error( ['ERROR (',mfilename '.m): Invalid second argument (''',args{2},''')!'] );
            end
            
        otherwise
            fcnNames = { 'qpOASES','qpOASES_sequence' };
            
    end
    
end


function [ timestamp ] = getTimestamp( dateString )

    try 
        timestamp = dateString.datenum;
    catch
        timestamp = Inf;
    end

end


function [ ] = printCopyrightNotice( )

    disp( ' ' );
    disp( 'qpOASES -- An Implementation of the Online Active Set Strategy.' );
    disp( 'Copyright (C) 2007-2017 by Hans Joachim Ferreau, Andreas Potschka,' );
    disp( 'Christian Kirches et al. All rights reserved.' );
    disp( ' ' );
    disp( 'qpOASES is distributed under the terms of the' );
    disp( 'GNU Lesser General Public License 2.1 in the hope that it will be' );
    disp( 'useful, but WITHOUT ANY WARRANTY; without even the implied warranty' );
    disp( 'of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.' );
    disp( 'See the GNU Lesser General Public License for more details.' );
    disp( ' ' );
    disp( ' ' );

end


%%
%%	end of file
%%
