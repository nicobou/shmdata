#!/usr/bin/python
"""
Release script usable by shmdata and switcher libraries.

Usage: ./release_version [OPTION]

Arguments:
  -l, --lib[=LIBRARY]    (Mandatory) Indicate which library will be released (must be shmdata or switcher)
"""
import sys
import getopt
import os
import shutil
import subprocess
import multiprocessing
import atexit
import re
import datetime

libs_root_path = os.path.join(os.path.expanduser("~"), 'src', 'releases')
libraries = ['shmdata', 'switcher']
version_file = 'CMakeLists.txt'

version_pattern = 'set\({}_VERSION_(\S+)\s+(\d+)\)'
shmdata_require_pattern = 'set\(SHMDATA_REQUIRED_VERSION (\d+\.\d+)\)'
github_path = 'git@github.com:sat-metalab'
remote_repo = 'origin'
bringup_branch = 'master'
working_branch = 'develop'
release_branch = 'release'
changelog_file = 'changelog'
success = True


def parse_version_number(lib, regex_pattern):
    config_file = os.path.join(libs_root_path, lib, version_file)
    version_number = [-1, -1, -1]
    with open(config_file) as file:
        major = minor = patch = -1
        for line in file:
            version_line = re.search(regex_pattern, line)
            if version_line and len(version_line.groups()) == 2:
                type = version_line.group(1)
                version = version_line.group(2)
                if type == 'MAJOR':
                    major = int(version)
                elif type == 'MINOR':
                    minor = int(version)
                elif type == 'PATCH':
                    patch = int(version)
                    break
            elif version_line and len(version_line.groups()) == 1:
                version = version_line.group(1).split('.')
                if len(version) == 2:
                    major = int(version[0])
                    minor = int(version[1])
                    patch = 0
                    break

    if major != -1 and minor != -1 and patch != -1:
        version_number = [major, minor, patch]
    else:
        printerr('Current version number not found in {}'.format(config_file))

    return version_number


def increase_version_number(version, version_increase):
    if version:
        if version_increase == 1:
            version[1] = 0
            version[2] = 0
        elif version_increase == 2:
            version[2] = 0
        version[version_increase - 1] += 1
    else:
        printerr('Invalid version number, cannot proceed to increase it.')


def update_switcher_shmdata_version():
    os.chdir(os.path.join(libs_root_path, 'shmdata'))
    assert git_checkout(bringup_branch) == 0, 'Failed to checkout {} branch of shmdata.'.format(bringup_branch)
    shmdata_version = parse_version_number('shmdata', version_pattern.format('SHMDATA'))
    assert shmdata_version != [-1, -1, -1]
    print 'found shmdata version {}.{}'.format(shmdata_version[0], shmdata_version[1])
    build_dir = os.path.join(libs_root_path, 'shmdata', 'build')
    if not os.path.isdir(build_dir):
        os.mkdir(build_dir)
    os.chdir(build_dir)
    if subprocess.call('cmake -DCMAKE_BUILD_TYPE=Release .. && make -j {} && sudo make install'
                               .format(multiprocessing.cpu_count()), shell=True) != 0:
        printerr('{} build failed, stopping the release.'.format(lib))
    lib_repo = '{}/{}.git'.format(github_path, lib)
    os.chdir(os.path.join(libs_root_path))
    assert git_clone(lib_repo) == 0, 'Could not fetch code base for library {} at {}'.format(lib, lib_repo)
    os.chdir(os.path.join(libs_root_path, lib))
    switcher_shmdata_version = parse_version_number(lib, shmdata_require_pattern)
    if switcher_shmdata_version[0] != shmdata_version[0] or switcher_shmdata_version[1] != shmdata_version[1]:
        assert git_checkout(working_branch) == 0, 'Failed to checkout {} branch of switcher.'.format(working_branch)
        commit_version_number(lib, shmdata_version, shmdata_require_pattern)


def commit_version_number(lib, new_version, regex_pattern):
    config_file = os.path.join(libs_root_path, lib, version_file)
    changed_file = config_file + ".tmp"
    shmdata_required_version = False
    with open(config_file) as old_file:
        with open(changed_file, 'w') as new_file:
            for line in old_file:
                version_line = re.search(regex_pattern, line)
                if version_line and len(version_line.groups()) == 2:
                    type = version_line.group(1)
                    if type == 'MAJOR':
                        line = re.sub(r'\d+', '{}'.format(new_version[0]), line)
                    elif type == 'MINOR':
                        line = re.sub(r'\d+', '{}'.format(new_version[1]), line)
                    elif type == 'PATCH':
                        line = re.sub(r'\d+', '{}'.format(new_version[2]), line)
                elif version_line and len(version_line.groups()) == 1:
                    version = version_line.group(1).split('.')
                    if len(version) == 2:
                        line = re.sub(r'\d+\.\d+', '{}.{}'.format(new_version[0], new_version[1]), line)
                        shmdata_required_version = True
                new_file.write(line)

    os.rename(changed_file, config_file)
    git_add([config_file])
    if shmdata_required_version:
        assert git_commit('Shmdata version change to {}.{}'.format(new_version[0], new_version[1])) == 0, \
                          'Failed to commit shmdata version number.'
    else:
        assert git_commit('Version {}.{}.{}'.format(new_version[0], new_version[1],
                                                    new_version[2])) == 0, 'Failed to commit version number.'


def git_push(remote_repo, remote_branch):
    return subprocess.call('git push {} {} --tags'.format(remote_repo, remote_branch), shell=True)


def git_checkout(branch_name, is_new=False):
    if is_new:
        return subprocess.call('git checkout -b {}'.format(branch_name), shell=True)
    else:
        return subprocess.call('git checkout {}'.format(branch_name), shell=True)


def git_commit(message):
    return subprocess.call('git commit -m "{}"'.format(message), shell=True)


def git_merge(branch_name, force=False):
    if force:
        return subprocess.call('git merge -X theirs {}'.format(branch_name), shell=True)
    else:
        return subprocess.call('git merge {}'.format(branch_name), shell=True)


def git_tag(tag_name):
    return subprocess.call('git tag {}'.format(tag_name), shell=True)


def git_add(file_list):
    for file in file_list:
        subprocess.call('git add {}'.format(file), shell=True)


def git_clone(repo_url):
    return subprocess.call('git clone {}'.format(repo_url), shell=True)


def git_pull():
    return subprocess.call('git pull', shell=True)


def get_git_config(property, default_value):
    config_property = default_value
    git_config_full = subprocess.check_output('git config --list', shell=True).strip().split('\n')
    for config in git_config_full:
        prop = config.split('=')
        if len(prop) < 2:
            break;
        if prop[0] == property:
            config_property = prop[1]
    return config_property


def update_changelog(lib, version):
    print 'Generating release notes'
    orig_file_name = 'NEWS.md'
    new_file_name = 'NEWS.md.new'
    authors_file_name = 'AUTHORS.md'

    git_checkout(working_branch)
    latest_tag = subprocess.check_output('git describe --tags --abbrev=0', shell=True)
    tag_date = subprocess.check_output('git log -1 --format=%ai {}'.format(latest_tag), shell=True)
    commits = re.split(r'commit [a-z0-9]+',
                       subprocess.check_output('git log --first-parent --since="{}"'.format(tag_date),
                                               shell=True).strip())
    with open(new_file_name, 'w') as new_file:
        with open(orig_file_name, 'r') as old_file:
            for i, line in enumerate(old_file.readlines()):
                if i != 3:
                    new_file.write(line)
                    continue
                new_file.write('\n{} {}.{}.{} ({})\n---------------------------\n'
                               'This is an official release in the {}.{} stable series.\n\n'
                               .format(lib, version[0], version[1], version[2], datetime.date.today(), version[0],
                                       version[1]))
                for commit in commits:
                    elements = commit.split('\n    ')
                    if len(elements) < 2:
                        continue
                    commit_message = ''
                    for message in elements[1:]:
                        commit_message += message + ' '
                    if commit_message:
                        commit_message = re.sub(r'reviewer\s*:\s*[a-z]+', r'', commit_message, flags=re.IGNORECASE)
                        new_file.write('* {}\n'.format(commit_message.strip()))

                new_file.write('\n')
    subprocess.call([get_git_config('core.editor', 'vim'), new_file_name])
    os.rename(new_file_name, orig_file_name)
    subprocess.call(os.path.join(sys.path[0], 'make_authors_from_git.sh'), shell=True)
    git_add([orig_file_name])
    if subprocess.call(os.path.join(sys.path[0], 'make_authors_from_git.sh'), shell=True) == 0:
        git_add([authors_file_name])
    git_commit('Updated changelog for version {}.{}.{}.'.format(version[0], version[1], version[2]))


@atexit.register  # Only clean on exit if the release was successful.
def cleanup_folder():
    global success
    if os.path.exists(libs_root_path) and success:
        shutil.rmtree(libs_root_path)
    success = False


def printerr(err):
    sys.stderr.write(err + '\n')
    exit(2)


def usage():
    print __doc__


if __name__ == '__main__':
    release_version = []
    version_increase = 0
    lib = ''

    try:
        opts, args = getopt.getopt(sys.argv[1:], "l", ["lib="])
    except getopt.GetoptError as err:
        print err
        usage()
        exit(2)

    for o, a in opts:
        if o in ["-l", "--lib"]:
            if a in libraries:
                lib = a
            else:
                usage()
                printerr('Unmanaged library {} provided'.format(a))
        else:
            usage()
            printerr('Unhandled option')

    if not lib:
        usage()
        printerr('Target library not provided')

    cleanup_folder()  # Always remove the folder when launching the script.
    os.mkdir(libs_root_path)
    os.chdir(libs_root_path)

    choice = raw_input('Is it a: 1/ Major release 2/ Minor release 3/ Bugfix release,'
                       ' of the {} library ? This will impact the new version number '
                       '(x.y.z matches the choices 1.2.3.): '.format(lib))
    if choice == '1':
        version_increase = 1
    elif choice == '2':
        version_increase = 2
    elif choice == '3':
        version_increase = 3
    else:
        printerr('Wrong choice. Aborting the release.')

    # Always clone shmdata even when releasing switcher to synchronize versions.
    lib_repo = '{}/{}.git'.format(github_path, 'shmdata')
    assert git_clone(lib_repo) == 0, 'Could not fetch code base for library {} at {}'.format(lib, lib_repo)

    if lib == 'switcher':
        update_switcher_shmdata_version()

    os.chdir(os.path.join(libs_root_path, lib))
    git_checkout(working_branch)

    release_version = parse_version_number(lib, version_pattern.format(lib.upper()))
    assert release_version != [-1, -1, -1]
    increase_version_number(release_version, version_increase)

    print 'Version number found for all libraries, now executing unit tests.'

    build_dir = os.path.join(libs_root_path, lib, 'build')

    if not os.path.isdir(build_dir):
        os.mkdir(build_dir)
    os.chdir(build_dir)

    if subprocess.call('cmake -DCMAKE_BUILD_TYPE=Release .. && make -j {} package_source_test'
                               .format(multiprocessing.cpu_count()), shell=True) != 0:
        printerr('{} unit tests failed, stopping the release.'.format(lib))

    os.chdir(os.path.join(libs_root_path, lib))

    print 'All unit tests passed successfully, now creating new branches for release.'

    update_changelog(lib, release_version)
    new_branch = '{}/version-{}.{}.{}'.format(release_branch, release_version[0], release_version[1],
                                              release_version[2])
    print 'Creating branch {} for release of {} library.'.format(new_branch, lib)
    git_checkout(new_branch, True)
    commit_version_number(lib, release_version, version_pattern.format(lib.upper()))
    assert git_checkout(bringup_branch) == 0, 'Could not checkout branch {}'.format(bringup_branch)
    assert git_merge(new_branch, True) == 0, 'Merge from branch {} into {} did not work.'.format(new_branch,
                                                                                                 bringup_branch)
    git_tag('{}.{}.{}'.format(release_version[0], release_version[1], release_version[2]))
    assert git_push(remote_repo, bringup_branch) == 0, 'Failed to push branch {} into {}/{}'.format(bringup_branch,
                                                                                                    remote_repo,
                                                                                                    bringup_branch)
    # If it's a bugfix release, we keep an odd number for develop and even for master.
    assert git_checkout(working_branch) == 0, 'Could not checkout branch {}'.format(working_branch)
    if version_increase == 3:
        release_version[2] += 1
    else:
        release_version[2] = 1
    commit_version_number(lib, release_version, version_pattern.format(lib.upper()))
    git_tag('{}.{}.{}'.format(release_version[0], release_version[1], release_version[2]))
    assert git_push(remote_repo, working_branch) == 0, 'Failed to push branch {} into {}/{}'.format(working_branch,
                                                                                                    remote_repo,
                                                                                                    working_branch)
    success = True
