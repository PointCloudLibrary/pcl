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
from urllib.parse import quote_plus

gh_auth = None


async def github_ratelimiter(headers):
    # If this is the last message before rate-limiting kicks in
    if int(headers['X-RateLimit-Remaining']) == 1:
        epoch_sec = int(headers['X-RateLimit-Reset'])
        delay = datetime.fromtimestamp(epoch_sec) - datetime.now()
        # adding 1 to ensure we wait till after the rate-limit reset
        sleep_time = delay.total_seconds() + 1
        print(f'Need to sleep for {sleep_time}')
        await asyncio.sleep(sleep_time)


async def get_issues(repository, closed=False, pull_request=False,
                     include_labels=[], exclude_labels=[],
                     sort='created', ascending_order=False):
    closed = 'closed' if closed else 'open'
    pull_request = 'pr' if pull_request else 'issue'
    print(f"Getting list of {closed} {pull_request} from GitHub")
    def gh_encode(x): return quote_plus(x, safe='"')

    api_url = f"https://api.github.com/search/issues?"
    # All query items are list of strings, which will be flattened later
    excluded_labels = [f'-label:"{gh_encode(x)}"' for x in exclude_labels]
    included_labels = [f'label:"{gh_encode(x)}"' for x in include_labels]
    issue = [f'is:{pull_request}']
    repo = [f"repo:{repository}"]
    status = [f"is:{closed}"]
    query = [issue, repo, status, excluded_labels, included_labels]

    query_string = 'q=' + '+'.join(itertools.chain.from_iterable(query))
    sort_string = f'sort={sort}'
    order_string = 'order=' + ('asc' if ascending_order else 'desc')

    query_url = api_url + '&'.join([query_string, sort_string, order_string])

    print(query_url)

    data_count = 0
    page = 1
    while True:
        # max pagination size is 100 as of github api v3
        search_url = f"{query_url}&page={page}&per_page=100"
        async with aiohttp.ClientSession() as session:
            response = await session.get(search_url, raise_for_status=True,
                                         headers=gh_auth)
            async with response:
                data = await response.json()
                total_count = data["total_count"]
                data_count += len(data["items"])
                for item in data["items"]:
                    yield item
                page += 1
                await github_ratelimiter(response.headers)
                # exit if all data has been received
                if data_count == total_count:
                    break
    print(f"Found {data_count} entries")


async def get_pr_details(issues):
    print("Getting more details about the PRs")
    counter = 0
    async for issue in issues:
        async with aiohttp.ClientSession() as session:
            response = await session.get(issue['pull_request']['url'],
                                         raise_for_status=True,
                                         headers=gh_auth)
            async with response:
                pr_data = await response.json()
                counter += 1
                yield pr_data
                await github_ratelimiter(response.headers)
    print(f"Received data about {counter} PRs")


async def pr_with_pending_review(pr_list, user):
    '''
    Generates PR which need to be reviewed by the user
    '''
    print(f"Filtering for @{user}")
    async for pr in pr_list:
        for reviewer in pr['requested_reviewers']:
            if reviewer['login'] == user:
                yield pr


def beautify_issues(github_issue_list):
    req_details = ["title", "body", "html_url", "created_at", "updated_at"]
    return [{x: issue[x] for x in req_details} for issue in github_issue_list]


def compose_message(issues):
    issue_data = [f'**{i+1}.** {issue["title"]}\n  {issue["html_url"]}'
                  for i, issue in enumerate(issues)]
    return '\n'.join(issue_data)


client = discord.Client()


async def set_playing(status):
    await client.change_presence(activity=discord.Game(name=status))


async def give_random(channel, number_of_issues):
    """
    @TODO: improve algorithm to save queries
    1. get first batch of github_max (100), find total number
    2. if total < requested, return all
    3. Generate requested random numbers
    4. get issues till max(generated_list)
    5. return them
    """
    await set_playing('Finding Issues')
    async with channel.typing():
        issues = [x async for x in get_issues('PointCloudLibrary/pcl',
                                              exclude_labels=['status: stale'])]
        reply = discord.Embed(color=discord.Color.purple())
        reply.title = f"{number_of_issues} random picks out of {len(issues)}:"

        chosen_issues = random.choices(issues, k=number_of_issues)
        reply.description = compose_message(beautify_issues(chosen_issues))
        if len(chosen_issues) < number_of_issues:
            reply.set_footer(text="There wasn't enough...")
    await channel.send(embed=reply)
    await set_playing('The Waiting Game')


async def review_q(channel, number_of_issues, author=None):
    await set_playing('On The Cue')
    async with channel.typing():
        issues = get_issues('PointCloudLibrary/pcl', pull_request=True,
                            exclude_labels=['status: stale'],
                            include_labels=['needs: code review'],
                            sort='updated', ascending_order=True)
        reply = discord.Embed(color=discord.Color.purple())

        if author:
            title_suffix = f" waiting @{author}'s review:"
            issues = pr_with_pending_review(get_pr_details(issues), author)
            chosen_issues = []
            # since async islice doesn't exist
            async for issue in issues:
                chosen_issues.append(issue)
                if len(chosen_issues) == number_of_issues:
                    break
        else:
            title_suffix = " in review queue:"
            chosen_issues = []
            async for issue in issues:
                chosen_issues.append(issue)
                if len(chosen_issues) == number_of_issues:
                    break

        reply.title = f"Oldest {number_of_issues} PR" + title_suffix
        reply.description = compose_message(beautify_issues(chosen_issues))
        if len(chosen_issues) < number_of_issues:
            reply.set_footer(text="There weren't enough...")
    await channel.send(embed=reply)
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
    if len(data) == 0 or data[0] != '!':
        return
    # split message into command and arguments
    query = data.strip().split(' ')
    command = query[0][1:]
    args = query[1:]

    reply = discord.Embed(color=discord.Color.purple())
    reply.description = "Talking to me? Use `!what` to know more."

    if command == 'what':
        reply.title = "Command list for GitHub Helper"
        reply.description = '''`!rand <N>`
Retrieves N random open, non-stale issues

`!review <N>`
Retrieves N least-recently-updated PR awaiting your review

`!q <N>`
Retrieves N least-recently-updated PR in the review queue'''
        await channel.send(embed=reply)
        return

    elif command == "rand":
        if len(args) != 1:
            await channel.send(embed=reply)
            return
        try:
            number_of_issues = int(args[0].strip())
            if number_of_issues < 1:
                raise ValueError("Positive integer needed")
        except ValueError:
            reply.description = "I can't give you un-natural issues." + \
                " I'm not a monster!!"
            await channel.send(embed=reply)
            return
        if number_of_issues > 10:
            number_of_issues = 10
            reply.description = "Let's curb that enthusiasm.. just a little"
            await channel.send(embed=reply)
        await give_random(channel, number_of_issues)
        return

    elif command == "q" or command == "review":
        if len(args) != 1:
            await channel.send(embed=reply)
            return
        try:
            number_of_issues = int(args[0].strip())
            if number_of_issues < 1:
                raise ValueError("Positive integer needed")
        except ValueError:
            await channel.send("This queue is 100% natural. Check your orders")
            return
        if number_of_issues > 10:
            number_of_issues = 10
            reply.description = "Let's curb that enthusiasm.. just a little"
            await channel.send(embed=reply)
        author = message.author
        author = None if command == "q" else (author.nick or author.name)
        await review_q(channel, number_of_issues, author)
        return
    return


def read_secret_token(filename):
    with open(filename, "r") as f:
        return f.readline().strip()


async def oneshot(channel_id, n):
    await client.wait_until_ready()
    await give_random(client.get_channel(channel_id), n)
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
    try:
        gh_token = read_secret_token(".github-token")
    except FileNotFoundError:
        gh_token = None
    global gh_auth
    gh_auth = {"Authorization": f"token {gh_token}"} if gh_token else None
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
    main()
