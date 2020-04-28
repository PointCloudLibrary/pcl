# Azure needs node shadow, sudo and the label
FROM node:lts-alpine

# clang-10 needed alpine edge as of 2020-Apr-28
RUN apk add \
        --no-cache \
        --repository=http://dl-cdn.alpinelinux.org/alpine/edge/main \
        bash clang git shadow sudo

LABEL "com.azure.dev.pipelines.agent.handler.node.path"="/usr/local/bin/node"

CMD [ "bash" ]
