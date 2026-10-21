"""Build the original executable's active geometry path without Windows SDKs.

The original EyeModelUpdater calls unproject_observations + initialise_model,
not Ceres/SPII refinement. Extract those exact definitions, omitting the unused
optimizer implementations and their dependencies. Original checkout is untouched.
"""
from pathlib import Path
import subprocess
import shutil

ROOT=Path(__file__).resolve().parents[2]
SOURCE=ROOT/'third_party/3D-Eye-Tracker'
REVISION='a8e2180577308596b6ac8078558df1e430789191'
OUT=ROOT/'build/itoh'

def main():
    actual=subprocess.check_output(['git','-C',str(SOURCE),'rev-parse','HEAD'],text=True).strip()
    if actual!=REVISION:raise RuntimeError('Unexpected upstream version: '+actual)
    OUT.mkdir(parents=True,exist_ok=True)
    for folder in ['main','singleeyefitter']:
        shutil.copytree(SOURCE/folder,OUT/folder,dirs_exist_ok=True)
    pupil=OUT/'main/pupilFitter.h'
    pupil.write_text(pupil.read_text().replace('getDarkestPixelBetter(Mat& I)', 'getDarkestPixelBetter(Mat I)'))
    ellipse=OUT/'singleeyefitter/singleeyefitter/Ellipse.h'
    ellipse.write_text(ellipse.read_text().replace('template<typename T>\n\t\tdouble similarity(const Ellipse2D<T>& e',
        'template<typename U>\n\t\tdouble similarity(const Ellipse2D<U>& e'))
    projection=OUT/'singleeyefitter/singleeyefitter/projection.h'
    s=projection.read_text()
    for condition in ['lambda(0) >= lambda(1)','lambda(1) > 0','lambda(2) < 0','n == 1']:
        s=s.replace('assert('+condition+');',
            'if (!('+condition+')) throw std::runtime_error("Degenerate pupil cone");')
    projection.write_text(s)
    text=(SOURCE/'singleeyefitter/SingleEyeFitter.cpp').read_text(encoding='utf-8-sig')
    def between(start,end):return text[text.index(start):text.index(end)]
    includes='\n'.join('#include <singleeyefitter/'+h+'.h>' for h in
        ['singleeyefitter','utils','projection','intersect','fun','math'])
    slim=includes+'\n#include "distance.h"\nnamespace singleeyefitter {\n'
    slim+=between('template<typename T>\nEigen::Matrix<T,3,1> sph2cart','template<typename T>\nT angleDiffGoodness')
    slim+=between('const EyeModelFitter::Vector3 EyeModelFitter::camera_centre','void singleeyefitter::EyeModelFitter::print_single_contrast_metric')
    # Constructors close namespace singleeyefitter upstream; qualified functions follow.
    slim=slim.replace('void EyeModelFitter::reset()', 'void singleeyefitter::EyeModelFitter::reset()')
    slim+=between('const singleeyefitter::EyeModelFitter::Circle& singleeyefitter::EyeModelFitter::initialise_single_observation(Pupil&',
                  'void singleeyefitter::EyeModelFitter::refine_with_inliers')
    slim+=text[text.index('void singleeyefitter::EyeModelFitter::initialise_model()'):]
    (OUT/'geometry.cpp').write_text(slim)
    updater=OUT/'main/eye_model_updater.cpp'
    s=updater.read_text().replace('sample_num_ = (w_num + 1)*(h_num + 1);',
        'sample_num_ = ((w + kSearchGridSize_ - 1)/kSearchGridSize_)*((h + kSearchGridSize_ - 1)/kSearchGridSize_);')
    s=s.replace('bb.contains(center)', 'bb.contains(cv::Point(center[0],center[1]))')
    s=s.replace('\n}\n\nEyeModelUpdater::EyeModelUpdater()', '\n\treturn false;\n}\n\nEyeModelUpdater::EyeModelUpdater()')
    updater.write_text(s)
    flags=subprocess.check_output(['pkg-config','--cflags','--libs','opencv4'],text=True).split()
    command=['g++','-std=c++14','-O2','-shared','-fPIC','-pthread',
        '-include',str(ROOT/'tools/itoh/compat.h'),'-I/usr/include/eigen3',
        '-I'+str(OUT/'main'),'-I'+str(OUT/'singleeyefitter'),
        str(ROOT/'tools/itoh/bridge.cpp'),str(OUT/'geometry.cpp'),str(updater),
        str(OUT/'singleeyefitter/utils.cpp'),'-o',str(OUT/'libitoh.so'),*flags]
    subprocess.run(command,check=True)
    shutil.copy(SOURCE/'LICENSE',OUT/'LICENSE')
    (OUT/'UPSTREAM.txt').write_text('https://github.com/YutaItoh/3D-Eye-Tracker\n'+REVISION+'\n')
    print(OUT/'libitoh.so')

if __name__=='__main__':main()
