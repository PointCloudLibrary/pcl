# escape=`

FROM mcr.microsoft.com/windows/servercore:ltsc2019

# Use "--build-arg platform=x64" for 64 bit or x86 for 32 bit.
ARG platform

# Download channel for fixed install.
ARG CHANNEL_URL=https://aka.ms/vs/16/release/channel
ADD ${CHANNEL_URL} C:\TEMP\VisualStudio.chman

COPY $platform'-windows-rel.cmake' 'c:\temp\'$platform'-windows-rel.cmake'

SHELL ["powershell", "-Command", "$ErrorActionPreference = 'Stop'; $ProgressPreference = 'SilentlyContinue';"]

# Download and install Build Tools for Visual Studio 2019 for native desktop
RUN wget 'https://aka.ms/vs/16/release/vs_buildtools.exe' -OutFile 'C:\TEMP\vs_buildtools.exe'; `
    Start-Process -FilePath C:\TEMP\vs_buildtools.exe -ArgumentList `
    "--quiet",                                                      `
    "--norestart",                                                  `
    "--nocache",                                                    `
    "--installPath",                                                `
    "C:\BuildTools",                                                `
    "--wait",                                                       `
    "--channelUri",                                                 `
    "C:\TEMP\VisualStudio.chman",                                   `
    "--installChannelUri",                                          `
    "C:\TEMP\VisualStudio.chman",                                   `
    "--add",                                                        `
    "Microsoft.VisualStudio.Workload.VCTools",                      `
    "--includeRecommended"                                          `
    -Wait -PassThru;                                                `
    del c:\temp\vs_buildtools.exe;                                  
    
RUN iex ((New-Object System.Net.WebClient).DownloadString('https://chocolatey.org/install.ps1'));        `
    choco install cmake git --installargs 'ADD_CMAKE_TO_PATH=System' -y --no-progress

RUN git clone https://github.com/microsoft/vcpkg.git;               ` 
    copy c:\temp\$Env:platform-windows-rel.cmake c:\vcpkg\triplets\$Env:platform-windows-rel.cmake;      `
    cd .\vcpkg;                                                `                                        `
    .\bootstrap-vcpkg.bat;                                            `
    .\vcpkg integrate install;                                        

RUN cd .\vcpkg;                                                `
    .\vcpkg install boost qhull flann gtest eigen3 vtk[qt,opengl] --triplet $Env:platform-windows-rel;             `
    Remove-Item -path c:\vcpkg\downloads -recurse -force; `
    Remove-Item -path c:\vcpkg\packages -recurse -force;  `
    Remove-Item -path c:\vcpkg\buildtrees -recurse -force; 

RUN setx path "C:\BuildTools\VC\Tools\MSVC\14.27.29110\bin\Host$Env:platform\$Env:platform;"
