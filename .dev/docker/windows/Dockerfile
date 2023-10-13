# escape=`

FROM mcr.microsoft.com/dotnet/framework/runtime:4.8.1

# Use "--build-arg platform=x64" for 64 bit or x86 for 32 bit.
ARG PLATFORM

# Use to set specific commit to checkout
ARG VCPKGCOMMIT

# Download channel for fixed install.
ARG CHANNEL_BASE_URL=https://aka.ms/vs/16/release

ADD $CHANNEL_BASE_URL/channel C:\TEMP\VisualStudio.chman

SHELL ["powershell", "-Command", "$ErrorActionPreference = 'Stop'; $ProgressPreference = 'SilentlyContinue';"]

# Download and install Build Tools for Visual Studio 2019 for native desktop
RUN wget $Env:CHANNEL_BASE_URL/vs_buildtools.exe -OutFile 'C:\TEMP\vs_buildtools.exe'; `
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
    "Microsoft.Net.Component.4.8.SDK",                            `
    "Microsoft.VisualStudio.Component.VC.ATLMFC",                   `
    "--includeRecommended"                                          `
    -Wait -PassThru;                                                `
    del c:\temp\vs_buildtools.exe;                                  

# VCPKG requires update if Cmake version is > 3.20.5 see: https://github.com/microsoft/vcpkg-tool/pull/107
RUN iex ((New-Object System.Net.WebClient).DownloadString('https://chocolatey.org/install.ps1'));   `
    choco install cmake --version=3.20.5 --installargs 'ADD_CMAKE_TO_PATH=System' -y --no-progress; `
    choco install git -y --no-progress

RUN git clone https://github.com/microsoft/vcpkg.git; cd vcpkg; git checkout $Env:VCPKGCOMMIT;

# To explicit set VCPKG to only build Release version of the libraries.
COPY $PLATFORM'-windows-rel.cmake' 'c:\vcpkg\triplets\'$PLATFORM'-windows-rel.cmake'

RUN cd .\vcpkg;                                                `
    .\bootstrap-vcpkg.bat;                                            `
    .\vcpkg install boost flann eigen3 qhull vtk[qt,opengl] gtest benchmark openni2 --triplet $Env:PLATFORM-windows-rel --host-triplet $Env:PLATFORM-windows-rel --clean-after-build --x-buildtrees-root=C:\b; `
    cat C:\b\qtdeclarative\install-$Env:PLATFORM-windows-rel-rel-out.log

RUN echo %ERRORLEVEL% `
    cmd.exe /C "cd /D C:\b\qtdeclarative\x86-windows-rel-rel\src\quick && C:\vcpkg\installed\x86-windows-rel\tools\Qt6\bin\qsb.exe --glsl 100es,120,150 --hlsl 50 --msl 12 -b -c -O -s -o C:/b/qtdeclarative/x86-windows-rel-rel/src/quick/.qsb/scenegraph/shaders_ng/flatcolor.vert.qsb C:/b/qtdeclarative/src/here-src-6-02b224fed5.clean/src/quick/scenegraph/shaders_ng/flatcolor.vert" `
    echo %ERRORLEVEL%
