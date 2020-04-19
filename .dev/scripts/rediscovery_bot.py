#! /usr/bin/env python3

"""
Software License Agreement (BSD License)

 Point Cloud Library (PCL) - www.pointclouds.org
 Copyright (c) 2020-, Open Perception, Inc.

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the copyright holder(s) nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.

"""

import aiohttp
import argparse
import asyncio
from datetime import datetime
import discord
import itertools
import random
import time


async def get_issues(repository, closed=False, include_labels=[],
                     exclude_labels=[]):
    print("Getting issues from GitHub")
    closed = 'closed' if closed else 'open'

    # All query items are list of strings, which will be flattened later
    status = [f"is:{closed}"]
    excluded_labels = [f"-label:'{x}'" for x in exclude_labels]
    included_labels = [f"label:'{x}'" for x in include_labels]
    repo = [f"repo:{repository}"]
    api_url = ["https://api.github.com/search/issues?q=is:issue"]
    queries = [api_url, repo, status, excluded_labels, included_labels]
    query_url = '+'.join(itertools.chain.from_iterable(queries))

    issue_data = []
    page = 1
    while True:
        # max pagination size is 100 as of github api v3
        search_url = f"{query_url}&page={page}&per_page=100"
        async with aiohttp.ClientSession() as session:
            response = await session.get(search_url, raise_for_status=True)
            async with response:
                data = await response.json()
                total_count = data["total_count"]
                issue_data.extend(data["items"])
                page += 1
                # implement rate limiting
                if int(response.headers['X-RateLimit-Remaining']) == 1:
                    epoch_sec = int(response.headers['X-RateLimit-Reset'])
                    delay = datetime.fromtimestamp(epoch_sec) - datetime.now()
                    # adding 1 to ensure we wait till after the rate-limit reset
                    await asyncio.sleep(delay.total_seconds() + 1)
                # exit if all data has been received
                if len(issue_data) == total_count:
                    break
    return issue_data


def beautify_issues(github_issue_list):
    req_details = ["title", "body", "html_url", "created_at", "updated_at"]
    return [{x: issue[x] for x in req_details} for issue in github_issue_list]


def compose_message(issues):
    message = [f"My top {len(issues)} picks are:"]
    issue_data = [f'{i+1}. **Issue:** {issue["title"]}\n  {issue["html_url"]}'
                  for i, issue in enumerate(issues)]
    return '\n'.join(itertools.chain.from_iterable([message, issue_data]))


client = discord.Client()


async def set_playing(status):
    await client.change_presence(activity=discord.Game(name=status))


async def send_message(channel, number_of_issues):
    await set_playing('Finding Issues')
    issues = await get_issues('PointCloudLibrary/pcl', exclude_labels=['stale'])

    async with channel.typing():
        chosen_issues = random.choices(issues, k=number_of_issues)
        reply = compose_message(beautify_issues(chosen_issues))
    await channel.send(reply)
    await set_playing('The Waiting Game')


@client.event
async def on_ready():
    await set_playing('The Waiting Game')


@client.event
async def on_message(message):
    # we do not want the bot to reply to itself
    if message.author.id == client.user.id:
        return
    # Don't reply to bots
    if message.author.bot:
        return
    channel = message.channel
    data = message.content
    if data[0] != '!':
        return
    # split message into command and arguments
    query = data.strip().split(' ')
    command = query[0][1:]
    args = query[1:]
    if command == 'what':
        await channel.send('Want to discover random open, non-stale issues?')
        await channel.send('Use `!give <N>` to get N picks made just for you')
        return

    if command == "give":
        if len(args) != 1:
            await channel.send("Talking to me? Use `!what` to know more.")
            return
        try:
            number_of_issues = int(args[0].strip())
            if number_of_issues < 1:
                raise ValueError("Positive integer needed")
        except ValueError:
            await channel.send("I can't give you un-natural issues."
                               " I'm not a monster!!")
            return

        await send_message(channel, number_of_issues)


def read_secret_token(filename):
    with open(filename, "r") as f:
        return f.readline().strip()


async def testing():
    print((await get_issues('PointCloudLibrary/pcl', exclude_labels=['stale']))[0])


async def oneshot(channel_id, n):
    await client.wait_until_ready()
    await send_message(client.get_channel(channel_id), n)
    await client.close()


def get_args():
    p = argparse.ArgumentParser("GitHub Issue Slot Machine",
                                description="""[Discord bot]
It helps to discover random open, non-stale issues.
By default, it'll enter interactive mode and return N issues when prompted by:
`!give N`
where N is a number less than open issues.
If a channel ID is provided, it'll send N issues and exit
""")
    p.add_argument("--channel_id", type=int,
                   help="Channel ID (numerical) to send messages to")
    p.add_argument("--issues", metavar="N", default=5, type=int,
                   help="Number of issues to send in one-shot mode, default: 5")
    return p.parse_known_args()


def main():
    args, _ = get_args()
    if not (args.channel_id and args.issues > 0):
        print("Entering interactive mode")
        client.run(read_secret_token(".discord-token"))
        return

    print("Running in one-shot mode."
          f" Will send {args.issues} messages to requested channel")
    loop = asyncio.get_event_loop()
    loop.run_until_complete(client.login(read_secret_token(".discord-token")))
    loop.create_task(oneshot(args.channel_id, args.issues))
    loop.run_until_complete(client.connect())


if __name__ == "__main__":
    """
    loop = asyncio.get_event_loop()
    loop.run_until_complete(testing())
    loop.close()
    """
    main()
