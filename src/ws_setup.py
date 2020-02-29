#
# Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this
# software and associated documentation files (the "Software"), to deal in the Software
# without restriction, including without limitation the rights to use, copy, modify,
# merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
# PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
# HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
# OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#

import boto3
import json
import sys
from time import gmtime, strftime, sleep
import random
import subprocess
import os
import yaml
import re

SYSTEM_INFO_FILE="./ws_info.yaml"
SYSTEM_INFO = [ "aws_region", 
                "ros_version",
                "gazebo_version",
                "project_dir",
                "vpc", 
                "security_groups", 
                "subnets",
                "iot_endpoint",
                "iot_credential_endpoint"
                ]

RESOURCE_SETTING_FILE="./ws_resources.yaml"
RESOURCES = [   "bucket_name", 
                "simulation_app_name", 
                "robot_app_name", 
                "iam_policy", 
                "iam_role", 
                "iot_thing",
                "iot_policy",
                "iot_certificate",
                "iot_iam_role",
                "iot_role_alias",
                "cognito_identity_pool_id"]

SETTING_FILE="./ws_settings.yaml"
SETTINGS = [    "iot_attach_principal_policy",
                "iot_attach_thing_principal",
                "iot_cert_settings",
                "sample_app_settings",
                "browser_ui_settings",
                "robomaker_settings"]

APPNAME_PREFIX="deliverychallenge"
BROWSER_UI_PATH="browser/js"
THING_CERT_DEST_PATH="robot_ws/src/deps/aws_game_manager/certs"
SAMPLE_APPLICATION_SETTINGS_PATH="robot_ws/src/delivery_robot_sample/settings"

class Setup:
    def __init__(self):
        self.settings = {}
        self.timestamp = strftime("%y%m%d-%H%M%S", gmtime())

    def create_cognito_identity_pool_id(self):
        log("check IdentityPoolID")
        if not self.settings["cognito_identity_pool_id"]:
            errlog("Setting for cognito_identity_pool_id couldn't be find.")
            errlog("This should be set manually in ws_resources.yaml file. You can use CloudFormation template Cognito_setting_CFn.yaml in browser/setup for setting up and having the value.")
            errlog("When you get IdentityPoolID through the CloudFormation stack, set the value into ws_resources.yaml file and execute this setup script again.")
            errlog("Please read README.md for more detail.")
            try:
                import webbrowser
                log("Opening AWS console for you..")
                webbrowser.open("https://console.aws.amazon.com/cloudformation/home")
            except:
                pass

            return None

    def create_iot_thing(self):
        log("create iot thing")
        client = boto3.client('iot')
        result = client.create_thing(thingName="{}_thing_{}".format(APPNAME_PREFIX, self.timestamp))    
        return result["thingName"]

    def create_iot_policy(self):
        log("create iot policy")
        client = boto3.client('iot')
        policyDocument = {
                "Version": "2012-10-17",
                "Statement": [{ 
                    "Effect": "Allow", 
                    "Action":["iot:*"], 
                    "Resource": ["*"] 
                }] 
            }
        result = client.create_policy(
            policyName="{}_thing_policy_{}".format(APPNAME_PREFIX, self.timestamp)  ,
            policyDocument=json.dumps(policyDocument)
        )
        return result["policyArn"]

    def create_iot_certificate(self):
        log("create iot create certificate")
        client = boto3.client('iot')
        result = client.create_keys_and_certificate(setAsActive=True)  
        cert_path = "{}/thing.cert.pem".format(THING_CERT_DEST_PATH)
        private_path = "{}/thing.private.key".format(THING_CERT_DEST_PATH)
        with open(cert_path, mode='w') as f:
            f.write(result["certificatePem"])
        with open(private_path, mode='w') as f:
            f.write(result["keyPair"]["PrivateKey"])
        return result["certificateArn"]

    def setup_iot_attach_principal_policy(self):
        log("attach printical to policy")
        client = boto3.client('iot')
        policy_arn = self.settings["iot_policy"]
        i = policy_arn.find(":policy/")
        policy_name = policy_arn[i+8:]
        response = client.attach_principal_policy(
            policyName=policy_name,
            principal=self.settings["iot_certificate"]
        ) 
        return True

    def setup_iot_attach_thing_principal(self):
        log("attach thing to principal")
        client = boto3.client('iot')
        response = client.attach_thing_principal(
            thingName=self.settings["iot_thing"],
            principal=self.settings["iot_certificate"]
        ) 
        return True

    def check_iot_endpoint(self):
        log("check iot endpoint")
        client = boto3.client('iot')
        response = client.describe_endpoint(endpointType="iot:Data-ATS")
        return response["endpointAddress"]

    def check_iot_credential_endpoint(self):
        log("check iot credential endpoint")
        client = boto3.client('iot')
        response = client.describe_endpoint(endpointType="iot:CredentialProvider")
        return response["endpointAddress"]

    def create_iot_iam_role(self):
        log("create iam role for IoT..")
        iam = boto3.client('iam')
        path='/'
        role_name = "Cloud9-%s-iot-iam-role-%s" % (APPNAME_PREFIX, self.timestamp) 
        log("iot iam role name : %s" % role_name)
        description='Role for RoboMaker workshop'
        trust_policy={
          "Version": "2012-10-17",
          "Statement": [
            {
              "Sid": "",
              "Effect": "Allow",
              "Principal": {
                "Service": "credentials.iot.amazonaws.com"
              },
              "Action": "sts:AssumeRole"
            }
          ]
        }
        response = iam.create_role(
            Path=path,
            RoleName=role_name,
            AssumeRolePolicyDocument=json.dumps(trust_policy),
            Description=description,
            MaxSessionDuration=3600,
        )
        result = response["Role"]["Arn"]
        iam.attach_role_policy(
            RoleName= role_name,
            PolicyArn= self.settings["iam_policy"]
        )
        return result

    def create_iot_role_alias(self):
        log("create iot role alias")
        role_alias_name = "Cloud9-{}_role_alias-{}".format(APPNAME_PREFIX, self.timestamp) 
        client = boto3.client('iot')
        response = client.create_role_alias(
            roleAlias=role_alias_name,
            roleArn=self.settings["iot_iam_role"])
        return response["roleAlias"]

    def check_project_dir(self):
        log("check project dir")
        result = os.getcwd().split("/")[-1]
        return result

    def check_aws_region(self):
        log("check region")
        session = boto3.session.Session()
        result = session.region_name
        return result

    def check_ros_version(self):
        log("check ROS version")
        ros_distro = os.environ['ROS_DISTRO']
        if ros_distro  == "kinetic":
            result = "Kinetic"
        elif ros_distro == "melodic":
            result = "Melodic"
        else:
            errlog("Colund't determin installed ros distribution. Only Melodic and Kinetic is supported")
            result = None
        return result

    def check_gazebo_version(self):
        log("decide gazebo")
        ros_distro = os.environ['ROS_DISTRO']
        if ros_distro  == "kinetic":
            result = 7
        elif ros_distro == "melodic":
                result = 9
        else:
            errlog("Colund't determin installed ros distribution. Only Melodic and Kinetic is supported")
            result = None
        return result        

    def create_simulation_app_name(self):
        log("create simulation app name")
        id = boto3.client('sts').get_caller_identity()
        arn = id["Arn"]
        i = arn.rfind('/')
        if i > 0:
            name = re.sub(r'[^a-zA-Z0-9_\-]', "-", arn[i+1:])
            result = "{}_sumulation_{}".format(APPNAME_PREFIX, name)
            if len(result) > 255:
                result = result[:255]
        else:
            result = "{}_sumulation".format(APPNAME_PREFIX)
        return result

    def create_robot_app_name(self):
        log("create robot app name")
        id = boto3.client('sts').get_caller_identity()
        arn = id["Arn"]
        i = arn.rfind('/')
        if i > 0:
            name = re.sub(r'[^a-zA-Z0-9_\-]', "-", arn[i+1:])
            result = "{}_robot_{}".format(APPNAME_PREFIX, name)
            if len(result) > 255:
                result = result[:255]
        else:
            result = "{}_robot".format(APPNAME_PREFIX)
        return result

    def check_vpc(self):
        log("check VPC")
        ec2 = boto3.client('ec2')
        result = [vpc['VpcId'] for vpc in ec2.describe_vpcs()['Vpcs'] if vpc["IsDefault"] == True][0]
        return result

    def check_security_groups(self):
        log("check security group")
        ec2 = boto3.client('ec2')
        result = [group["GroupId"] for group in ec2.describe_security_groups()['SecurityGroups'] \
            if 'VpcId' in group and group["GroupName"] == "default" and group["VpcId"] == self.settings["vpc"]]
        return result
        
    def check_subnets(self):
        log("check subnet")
        ec2 = boto3.client('ec2')
        result = [subnet["SubnetId"] for subnet in ec2.describe_subnets()["Subnets"] \
                    if subnet["VpcId"] == self.settings["vpc"] and subnet['DefaultForAz']==True]
        return result

    def create_bucket_name(self):
        log("create bucket name")
        done = False
        retry = 5
        id = boto3.client('sts').get_caller_identity()
        accountId = id["Account"]            
        while(not done):
            result = "%s-%s-%s-%s" % (APPNAME_PREFIX, self.settings["aws_region"], accountId, self.timestamp)  
            
            log("Create S3 bucket: %s.." % result)
            #create bucket 
            try: 
                region_name = self.settings["aws_region"]
                s3_client = boto3.client('s3', region_name=region_name)
                if region_name == "us-east-1":
                    s3_client.create_bucket(Bucket=result)
                else:
                    location = {'LocationConstraint': self.settings["aws_region"]}
                    s3_client.create_bucket(Bucket=result, CreateBucketConfiguration=location)
            except Exception as e:
                retry -= 1
                if retry >= 0:
                    log(" => Failed.. retrying")
                    sleep(random.randint(1,3))
                    continue
                else:
                    errlog("Failed to create S3 bucket!")
                    errlog("Error Message: %s" % str(e))
                    return None
            done = True
        return result
        
    def create_iam_policy(self):
        log("create iam policy..")
        iam = boto3.client('iam')
        id = boto3.client('sts').get_caller_identity()
        accountId = id["Account"]

        policy_name = "%s-%s" % (APPNAME_PREFIX, self.timestamp) 
        log("policy name : %s" % policy_name)

        my_policy = {
                "Version": "2012-10-17",
                "Statement": [
                {
                    "Action": "s3:*",
                    "Resource": [
                        "*"
                    ],
                    "Effect": "Allow"
                },
                {
                    "Action": [
                        "logs:CreateLogGroup",
                        "logs:CreateLogStream",
                        "logs:PutLogEvents",
                        "logs:DescribeLogStreams"
                    ],
                    "Resource": [
                        "arn:aws:logs:*:{}:log-group:/aws/robomaker/SimulationJobs*".format(accountId)
                    ],
                    "Effect": "Allow"
                },
                {
                    "Action": [
                        "cloudwatch:PutMetricData",
                    ],
                    "Effect": "Allow",
                    "Resource": "*"
                },
                {
                    "Action": [
                        "robomaker:TagResource",
                        "robomaker:UntagResource",
                        "robomaker:ListTagsForResource",
                        "robomaker:CancelSimulationJob"
                    ],
                    "Resource": [
                        "arn:aws:robomaker:*:{}:simulation-job*".format(accountId)
                    ],
                    "Effect": "Allow"
                }
            ]
        }
        
        response = iam.create_policy(
            PolicyName=policy_name,
            PolicyDocument=json.dumps(my_policy)
        )
        result = response["Policy"]["Arn"]            
        return result

    def create_iam_role(self):
        log("create iam role..")
        iam = boto3.client('iam')
        path='/'
        role_name = "Cloud9-%s-sim-%s" % (APPNAME_PREFIX, self.timestamp) 
        log("role name : %s" % role_name)
        description='Role for RoboMaker workshop'

        trust_policy={
          "Version": "2012-10-17",
          "Statement": [
            {
              "Sid": "",
              "Effect": "Allow",
              "Principal": {
                "Service": "robomaker.amazonaws.com"
              },
              "Action": "sts:AssumeRole"
            }
          ]
        }
        
        response = iam.create_role(
            Path=path,
            RoleName=role_name,
            AssumeRolePolicyDocument=json.dumps(trust_policy),
            Description=description,
            MaxSessionDuration=3600,
        )

        result = response["Role"]["Arn"]

        iam.attach_role_policy(
            RoleName= role_name,
            PolicyArn= self.settings["iam_policy"]
        )
        return result

    def setup_iot_cert_settings(self):
        log("update {}/config.yaml.temp".format(THING_CERT_DEST_PATH))
        self.update_setting_file("templates/config.yaml.temp" , "{}/config.yaml".format(THING_CERT_DEST_PATH))
        return True

    def setup_browser_ui_settings(self):
        log("update {}/app-settings.js".format(BROWSER_UI_PATH))
        self.update_setting_file("templates/app-settings.js.temp" , "{}/app-settings.js".format(BROWSER_UI_PATH))
        return True

    def setup_robomaker_settings(self):
        log("setup roboMakerSettings.json..")
        self.update_setting_file("templates/roboMakerSettings.temp"  ,"../roboMakerSettings.json" )
        return True
    
    def setup_sample_app_settings(self):
        log("setup sample app setting file settings.yaml..")
        self.update_setting_file("templates/settings.yaml.temp"  ,"{}/settings.yaml".format(SAMPLE_APPLICATION_SETTINGS_PATH))
        return True


    def update_setting_file(self, in_file, out_file):
        with open(in_file) as f:
            lines = f.read()

        for item in self.settings:
            value = str(self.settings[item])
            value = value.replace("'", "\"")
            lines = lines.replace("<{}>".format(item), value)

        with open(out_file, mode="w") as f:
            f.write(lines)        

    def postProcess(self):
        try:
            log("Perform the build and bundles...")
            log("Setp 1. simulation_ws install dependencies...")
            os.chdir(os.path.abspath('./simulation_ws'))
            result = subprocess.call("rosdep install --from-paths src --ignore-src -r -y".split())
            if result == 0:
                log(" =>OK")
            else:
                errlog("Setup failed!")
                errlog("Reason: Failed to bundle the sample program!")
                sys.exit(1)
                
            log("Setp 2. simulation_ws build...")
            result = subprocess.call("colcon build".split())
            if result == 0:
                log(" =>OK")
            else:
                errlog("Setup failed!")
                errlog("Reason: Failed to build the sample program!")
                sys.exit(1)
                
            log("Setp 3. simulation_ws bundle...")
            result = subprocess.call("colcon bundle".split())
            if result == 0:
                log(" =>OK")
            else:
                errlog("Setup failed!")
                errlog("Reason: Failed to bundle the sample program!")
                sys.exit(1)

            log("Setp 4. robot_ws install dependencies...")
            os.chdir(os.path.abspath('../robot_ws'))
            result = subprocess.call("rosdep install --from-paths src --ignore-src -r -y".split())
            if result == 0:
                log(" =>OK")
            else:
                errlog("Setup failed!")
                errlog("Reason: Failed to bundle the sample program!")
                sys.exit(1)
                
            log("Setp 5. robot_ws build...")
            result = subprocess.call("colcon build".split())
            if result == 0:
                log(" =>OK")
            else:
                errlog("Setup failed!")
                errlog("Reason: Failed to build the sample program!")
                sys.exit(1)
                
            log("Setp 6. robot_ws bundle...")
            result = subprocess.call("colcon bundle".split())
            if result == 0:
                log(" =>OK")
            else:
                errlog("Setup failed!")
                errlog("Reason: Failed to bundle the sample program!")
                sys.exit(1)
                
        except Exception as e:
            errlog("Exception : %s" % str(e))
            sys.exit(1)

    def saveSettings(self, _settings, items, file_name):
        if not file_name:
            return

        for aSetting in items:
            if not aSetting in _settings:
                _settings[aSetting] = None
        
        f = open(file_name, "w+")
        f.write(yaml.dump(_settings, default_flow_style=False))
    
    def entry(self, items, prefix, file_name = None):
        if file_name and os.path.exists(file_name):
            try:
                f = open(file_name, "r+")
                _settings = yaml.load(f, Loader=yaml.FullLoader)
                if _settings:
                    self.settings.update(_settings)
                else:
                    _settings = {}
            except Exception as e:
                errlog("\nSetup failed! \n => Reason: Setting file %s exists but failed to load\n" % file_name)
                errlog(" => Error Message: %s\n\n" % str(e))
                sys.exit(1)
        else:
            _settings = {}

        for aSetting in items:
            print("Check %s" % aSetting)
            try:            
                if (not aSetting in _settings) or (not _settings[aSetting]) :
                    func_name = "%s_%s" % (prefix, aSetting)
                    result = getattr(self, func_name)()
                    if not result:
                        errlog("Failed to setup %s\nFinishing...\n" % aSetting)
                        self.saveSettings(_settings, items, file_name)
                        sys.exit(1)
                    _settings[aSetting] = result
                    self.settings.update({aSetting : result})
            except Exception as e:
                errlog("Exception : %s" % str(e))
                errlog("Failed to setup %s\nFinishing...\n" % aSetting)
                self.saveSettings(_settings, items, file_name)
                sys.exit(1)
                
            print("   => Ok")
            print("   Using %s for %s" % (str(_settings[aSetting]),aSetting))
        self.saveSettings(_settings, items, file_name)
        
def log(message):
    print("  \033[34m{}\033[0m".format(message))

def errlog(message):
    print("\033[91m{}\033[0m".format(message))

if __name__ == '__main__':
    setup = Setup()
    print("Start setup process..")
    print("Setup-1. Check system information.")
    setup.entry(SYSTEM_INFO, "check", SYSTEM_INFO_FILE)
    print("Setup-2. Create AWS resources.")
    setup.entry(RESOURCES, "create", RESOURCE_SETTING_FILE)
    print("Setup-3. Update configuration files.")
    setup.entry(SETTINGS, "setup", SETTING_FILE)
    print("Setup finished successfully!")    
    print("Execute the post process..")
    setup.postProcess()
